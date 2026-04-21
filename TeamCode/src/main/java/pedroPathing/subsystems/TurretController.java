package pedroPathing.subsystems;

import static com.pedropathing.pathgen.MathFunctions.clamp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * TurretController with tick-based soft limits and a hold-at-target feature.
 * Use setHoldAtZeroEnabled(true) and setHoldTargetTick(...) to lock turret.
 */
public class TurretController {

    public static final double GOAL_X = 12.5;
    public static final double GOAL_Y = 137.5;

    public static final double TURRET_TICKS_PER_REV = 2688.5;
    public static final double TURRET_TICKS_PER_RAD = TURRET_TICKS_PER_REV / (2.0 * Math.PI);

    private static double mountOffsetRad = 0.0;
    private static int headingSign = -1;

    private static boolean swapXY = false;
    private static boolean invertX = true;
    private static boolean invertY = false;

    private boolean tickSoftLimitsEnabled = false;
    private int minTick = -1650;
    private int maxTick = 1025;
    private int softMarginTicks = 3;
    private int slowZoneTicks = 15;
    private double slowZoneMinScale = 0.3;

    public static boolean USE_SOFT_LIMITS = false;
    public static double MIN_ANGLE_RAD = Math.toRadians(-270);
    public static double MAX_ANGLE_RAD = Math.toRadians(270);

    private double kP = 0.6;
    private double kD = 0.004;
    private double kS = 0.09;
    private double maxPower = 1.0;
    private double deadbandRad = Math.toRadians(1.5);

    private final DcMotorEx turretMotor;
    private final Follower follower;

    private double lastError = 0.0;
    private long lastTimeNanos = 0L;

    // Hold mode
    private boolean holdAtZero = false;
    private int holdTargetTick = 0;
    private double hold_kP = 0.9;
    private double hold_kD = 0.01;
    private double hold_kS = 0.12;

    public TurretController(HardwareMap hw, String turretMotorName, Follower follower) {
        this.turretMotor = hw.get(DcMotorEx.class, turretMotorName);
        this.follower = follower;
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastTimeNanos = System.nanoTime();
    }

    private static double wrapRad(double a) {
        while (a >= Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }

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

    public void setTickSoftLimitsEnabled(boolean enabled) {
        this.tickSoftLimitsEnabled = enabled;
    }
    public void setTickLimits(int minTick, int maxTick) {
        if (minTick > maxTick) {
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

    // Hold API
    public void setHoldAtZeroEnabled(boolean enabled) {
        this.holdAtZero = enabled;
        if (enabled) {
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lastTimeNanos = System.nanoTime();
        }
    }
    public void setHoldTargetTick(int tick) {
        this.holdTargetTick = tick;
    }
    public void setHoldGains(double kP, double kD, double kS) {
        this.hold_kP = kP; this.hold_kD = kD; this.hold_kS = kS;
    }

    /** Expose current encoder tick for callers that want to lock to current position. */
    public int getCurrentTick() {
        return turretMotor.getCurrentPosition();
    }

    /** Call every loop AFTER follower.update(). */
    public void update() {
        Pose pose = follower.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();

        double fieldAngleToGoal = mappedFieldAngleToGoal(robotX, robotY);
        double desiredTurretAngleRad = wrapRad(fieldAngleToGoal - (headingSign * robotHeading) - mountOffsetRad);

        if (holdAtZero) {
            desiredTurretAngleRad = (holdTargetTick / TURRET_TICKS_PER_RAD);
        }

        if (USE_SOFT_LIMITS) {
            desiredTurretAngleRad = clamp(desiredTurretAngleRad, MIN_ANGLE_RAD, MAX_ANGLE_RAD);
        }

        final int currentTicks = turretMotor.getCurrentPosition();
        double currentAngleRad = currentTicks / TURRET_TICKS_PER_RAD;

        if (tickSoftLimitsEnabled) {
            int desiredTicks = (int) Math.round(desiredTurretAngleRad * TURRET_TICKS_PER_RAD);
            desiredTicks = (int) clamp(desiredTicks, minTick, maxTick);
            desiredTurretAngleRad = desiredTicks / TURRET_TICKS_PER_RAD;

            double error = desiredTurretAngleRad - currentAngleRad;
            if (Math.abs(error) <= deadbandRad) {
                turretMotor.setPower(0.0);
                lastError = error;
                lastTimeNanos = System.nanoTime();
                return;
            }

            long now = System.nanoTime();
            double dt = Math.max(1e-4, (now - lastTimeNanos) / 1e9);
            double deriv = (error - lastError) / dt;

            double use_kP = holdAtZero ? hold_kP : kP;
            double use_kD = holdAtZero ? hold_kD : kD;
            double use_kS = holdAtZero ? hold_kS : kS;

            double power = use_kP * error + use_kD * deriv + use_kS * Math.signum(error);

            if ((currentTicks - minTick) <= softMarginTicks && power < 0) power = 0.0;
            if ((maxTick - currentTicks) <= softMarginTicks && power > 0) power = 0.0;

            int distToMin = Math.max(0, currentTicks - minTick);
            int distToMax = Math.max(0, maxTick - currentTicks);
            int distToNearest = Math.min(distToMin, distToMax);
            if (distToNearest <= slowZoneTicks && distToNearest > softMarginTicks) {
                double t = (distToNearest - softMarginTicks) / (double) Math.max(1, slowZoneTicks - softMarginTicks);
                double scale = slowZoneMinScale + (1.0 - slowZoneMinScale) * t;
                power *= clamp(scale, slowZoneMinScale, 1.0);
            }

            if (currentTicks < minTick) power = Math.max(power, +0.2);
            else if (currentTicks > maxTick) power = Math.min(power, -0.2);

            power = clamp(power, -maxPower, +maxPower);
            turretMotor.setPower(power);
            lastError = error;
            lastTimeNanos = now;
            return;
        }

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

        double use_kP = holdAtZero ? hold_kP : kP;
        double use_kD = holdAtZero ? hold_kD : kD;
        double use_kS = holdAtZero ? hold_kS : kS;

        double power = use_kP * error + use_kD * deriv + use_kS * Math.signum(error);
        power = clamp(power, -maxPower, +maxPower);
        turretMotor.setPower(power);
        lastError = error;
        lastTimeNanos = now;
    }

    public double getMountOffsetRad() { return mountOffsetRad; }
    public int getHeadingSign() { return headingSign; }
    public boolean getSwapXY() { return swapXY; }
    public boolean getInvertX() { return invertX; }
    public boolean getInvertY() { return invertY; }

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
