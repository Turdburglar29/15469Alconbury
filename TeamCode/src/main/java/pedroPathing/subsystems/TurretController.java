
package pedroPathing.subsystems;

import static com.pedropathing.pathgen.MathFunctions.clamp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * PID turret controller that points at a fixed field goal using Pedro Pathing pose.53.5
 * Now includes tick-based soft limits (min/max encoder counts) with soft margin and slow zone.
 */
public class TurretController {

    // === Field Goal (inches) ===
    public static final double GOAL_X = 12.5;
    public static final double GOAL_Y = 137.5;

    // === Turret gearing / encoder ===
    // 145.1 ticks/rev at motor; 70/14 = 5:1 -> 537.7 * 5 = 725.5 ticks per 360°
    public static final double TURRET_TICKS_PER_REV = 2688.5; // encoder ticks to turn 360°
    public static final double TURRET_TICKS_PER_RAD = TURRET_TICKS_PER_REV / (2.0 * Math.PI);

    // === Configurables ===
    private static double mountOffsetRad = 0.0; // encoder 0 vs robot-forward (+X), + = CCW from forward
    private static int headingSign = -1;        // -1 = heading is CW-positive; +1 = CCW-positive

    // --- Field-frame correction switches ---
    private static boolean swapXY = false;
    private static boolean invertX = true;
    private static boolean invertY = false;

    // === NEW: Tick-based soft-limit configuration ===
    private boolean tickSoftLimitsEnabled = false;
    private int minTick = -1650;           // counter-clockwise side (do not go less than this)
    private int maxTick = 1025;            // clockwise side (do not go more than this)
    private int softMarginTicks = 3;      // within this many ticks of a boundary: no outbound power
    private int slowZoneTicks = 15;       // within this many ticks: scale power down
    private double slowZoneMinScale = 0.3; // min power scale at boundary

    // (Angle-based limits are still available if you want them)
    public static boolean USE_SOFT_LIMITS = false;
    public static double MIN_ANGLE_RAD = Math.toRadians(-270);
    public static double MAX_ANGLE_RAD = Math.toRadians(270);

    // PID(+S) gains
    private double kP = 0.6;
    private double kD = 0.004;
    private double kS = 0.09;
    private double maxPower = 1.0;
    private double deadbandRad = Math.toRadians(1.5);

    // Hardware
    private final DcMotorEx turretMotor;
    private final Follower follower;

    // State (for derivative)
    private double lastError = 0.0;
    private long lastTimeNanos = 0L;

    public TurretController(HardwareMap hw, String turretMotorName, Follower follower) {
        this.turretMotor = hw.get(DcMotorEx.class, turretMotorName);
        this.follower = follower;
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastTimeNanos = System.nanoTime();
    }

    /** Angle wrap to [-π, π). */
    private static double wrapRad(double a) {
        while (a >= Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

    // ----------- Field-frame mapping helpers -----------
    private static double mapX(double x, double y) {
        double rx = swapXY ? y : x;
        if (invertX) rx = -rx;
        return rx;
    }
    private static double mapY(double x, double y) {
        double ry = swapXY ? x : y;
        if (invertY) ry = -ry;
        return ry;
    }
    private static double mappedFieldAngleToGoal(double robotX, double robotY) {
        double rx = mapX(robotX, robotY);
        double ry = mapY(robotX, robotY);
        double rgx = mapX(GOAL_X, GOAL_Y);
        double rgy = mapY(GOAL_X, GOAL_Y);
        return Math.atan2(rgy - ry, rgx - rx);
    }

    // ----------- Public configuration -----------
    public void setInverted(boolean inverted) {
        turretMotor.setDirection(inverted ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }
    public void setHeadingCcwPositive(boolean ccwPositive) {
        headingSign = ccwPositive ? +1 : -1;
    }
    public void setMountOffsetRad(double offsetRad) {
        mountOffsetRad = offsetRad;
    }
    public void setSwapXY(boolean v) { swapXY = v; }
    public void setInvertX(boolean v) { invertX = v; }
    public void setInvertY(boolean v) { invertY = v; }
    public void setGains(double kP, double kD, double kS) {
        this.kP = kP; this.kD = kD; this.kS = kS;
    }
    public void setMaxPower(double maxPower) {
        this.maxPower = Math.max(0.1, Math.min(1.0, maxPower));
    }
    public void setDeadbandDeg(double deg) {
        this.deadbandRad = Math.toRadians(Math.max(0.2, Math.min(10.0, deg)));
    }

    // === NEW: Tick-based soft-limit setters (used by your TeleOp) ===
    public void setTickSoftLimitsEnabled(boolean enabled) {
        this.tickSoftLimitsEnabled = enabled;
    }
    public void setTickLimits(int minTick, int maxTick) {
        if (minTick > maxTick) {
            // Swap if given backwards
            int tmp = minTick; minTick = maxTick; maxTick = tmp;
        }
        this.minTick = minTick;
        this.maxTick = maxTick;
    }
    public void setSoftMarginTicks(int softMarginTicks) {
        this.softMarginTicks = Math.max(0, softMarginTicks);
    }
    public void setSlowZoneTicks(int slowZoneTicks) {
        this.slowZoneTicks = Math.max(0, slowZoneTicks);
    }

    /** Call every loop AFTER follower.update(). */
    public void update() {
        Pose pose = follower.getPose(); // inches / radians
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading(); // (CW-positive on your robot if headingSign=-1)

        // Field-relative angle to goal
        double fieldAngleToGoal = mappedFieldAngleToGoal(robotX, robotY);

        // Desired turret angle in radians (robot-relative)
        double desiredTurretAngleRad =
                wrapRad(fieldAngleToGoal - (headingSign * robotHeading) - mountOffsetRad);

        // Optional angle-based limits (legacy)
        if (USE_SOFT_LIMITS) {
            desiredTurretAngleRad = clamp(desiredTurretAngleRad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
        }

        // Current turret state
        final int currentTicks = turretMotor.getCurrentPosition();
        double currentAngleRad = currentTicks / TURRET_TICKS_PER_RAD;

        // === Tick soft limits (primary protection) ===
        if (tickSoftLimitsEnabled) {
            // 1) Clamp the desired target to the legal tick window so we never command past it.
            int desiredTicks = (int) Math.round(desiredTurretAngleRad * TURRET_TICKS_PER_RAD);
            desiredTicks = (int) clamp(desiredTicks, minTick, maxTick);
            desiredTurretAngleRad = desiredTicks / TURRET_TICKS_PER_RAD;

            // 2) Compute plain error (no wrap) so we don't try to "wrap through" the limits.
            double error = desiredTurretAngleRad - currentAngleRad;

            // Deadband: hold position when aimed
            if (Math.abs(error) <= deadbandRad) {
                turretMotor.setPower(0.0);
                lastError = error;
                lastTimeNanos = System.nanoTime();
                return;
            }

            // PD + kS
            long now = System.nanoTime();
            double dt = Math.max(1e-4, (now - lastTimeNanos) / 1e9);
            double deriv = (error - lastError) / dt;
            double power = kP * error + kD * deriv + kS * Math.signum(error);

            // 3) Outbound motion suppression inside soft margin (no pushing into the stop).
            //    - Outbound toward minTick means negative power when we're near min.
            //    - Outbound toward maxTick means positive power when we're near max.
            if ((currentTicks - minTick) <= softMarginTicks && power < 0) {
                power = 0.0; // block CCW beyond min
            }
            if ((maxTick - currentTicks) <= softMarginTicks && power > 0) {
                power = 0.0; // block CW beyond max
            }

            // 4) Slow zone scaling as we approach the limits (linear scale).
            int distToMin = Math.max(0, currentTicks - minTick);
            int distToMax = Math.max(0, maxTick - currentTicks);
            int distToNearest = Math.min(distToMin, distToMax);
            if (distToNearest <= slowZoneTicks && distToNearest > softMarginTicks) {
                double t = (distToNearest - softMarginTicks) / (double) Math.max(1, slowZoneTicks - softMarginTicks);
                double scale = slowZoneMinScale + (1.0 - slowZoneMinScale) * t; // from minScale at edge -> 1.0 away from edge
                power *= clamp(scale, slowZoneMinScale, 1.0);
            }

            // 5) If we ever get outside the limits (e.g., brownout or slip), gently drive back in.
            if (currentTicks < minTick) {
                power = Math.max(power, +0.2); // force inward (positive) toward range
            } else if (currentTicks > maxTick) {
                power = Math.min(power, -0.2); // force inward (negative) toward range
            }

            // Final clamp & send
            power = clamp(power, -maxPower, +maxPower);
            turretMotor.setPower(power);
            lastError = error;
            lastTimeNanos = now;
            return;
        }

        // === Original (wrap-around) controller path if tick soft limits are disabled ===
        double error = wrapRad(desiredTurretAngleRad - currentAngleRad);

        if (Math.abs(error) <= deadbandRad) {
            turretMotor.setPower(0.0);
            lastError = error;
            lastTimeNanos = System.nanoTime();
            return;
        }

        long now = System.nanoTime();
        double dt = Math.max(1e-4, (now - lastTimeNanos) / 1e9);
        double deriv = (error - lastError) / dt;
        double power = kP * error + kD * deriv + kS * Math.signum(error);
        power = clamp(power, -maxPower, +maxPower);
        turretMotor.setPower(power);
        lastError = error;
        lastTimeNanos = now;
    }

    // --- Accessors for telemetry ---
    public double getMountOffsetRad() { return mountOffsetRad; }
    public int getHeadingSign() { return headingSign; }
    public boolean getSwapXY() { return swapXY; }
    public boolean getInvertX() { return invertX; }
    public boolean getInvertY() { return invertY; }

    /** Calibrate mount offset so that the current physical aim is considered "on target". */
    public void calibrateMountOffsetToCurrentAim() {
        Pose pose = follower.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();

        double fieldAngleToGoal = mappedFieldAngleToGoal(robotX, robotY);

        int ticks = turretMotor.getCurrentPosition();
        double currentAngleRad = ticks / TURRET_TICKS_PER_RAD;

        double newOffset = wrapRad(fieldAngleToGoal - (headingSign * robotHeading) - currentAngleRad);
        mountOffsetRad = newOffset;
    }

    public static double wrapForTelemetry(double a) { return wrapRad(a); }
    public static double mappedFieldAngleForTelemetry(double robotX, double robotY) {
        return mappedFieldAngleToGoal(robotX, robotY);
    }
}
