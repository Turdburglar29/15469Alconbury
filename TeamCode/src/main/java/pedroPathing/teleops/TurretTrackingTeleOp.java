package pedroPathing.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "TurretTrackingTeleOp", group = "Competition")
public class TurretTrackingTeleOp extends OpMode {

    /* ================= HARDWARE ================= */
    private DcMotor turret;
    private Follower follower;

    /* ================= CONSTANTS ================= */

    // goBILDA 312 RPM encoder
    static final double TICKS_PER_REV = 537.6898;

    // Gear ratio: 14T motor -> 62T turret
    static final double GEAR_RATIO = 62.0 / 14.0;

    static final double TICKS_PER_RAD =
            (TICKS_PER_REV * GEAR_RATIO) / (2.0 * Math.PI);

    /* -------- Turret range -------- */
    static final double MIN_TURRET_RAD = 0.0;
    static final double MAX_TURRET_RAD = 2.0 * Math.PI;

    /* -------- Forbidden zone -------- */
    static final double FORBIDDEN_START = Math.toRadians(170);
    static final double FORBIDDEN_END   = Math.toRadians(190);

    /* -------- Encoder hard limits -------- */
    static final int TURRET_MIN_TICKS = 0;
    static final int TURRET_MAX_TICKS =
            (int) (MAX_TURRET_RAD * TICKS_PER_RAD);

    /* -------- Control -------- */
    static final double kP = 4.8;
    static final double kS = 0.04;
    static final double MAX_POWER = 0.45;
    static final double ANGLE_DEADBAND = Math.toRadians(1.0);
    static final double FILTER_ALPHA = 0.10;

    /* -------- Decode goal (field coords) -------- */
    static final double GOAL_X = 12.5;
    static final double GOAL_Y = 137.5;

    /* ================= STATE ================= */
    private double filteredTargetAngle = 0.0;

    /* ================= INIT ================= */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(75, 75, 0));

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Turret Tracking Ready");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        filteredTargetAngle = 0.0;
    }

    /* ================= LOOP ================= */
    @Override
    public void loop() {

        /* -------- Drive -------- */
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
        follower.update();

        /* -------- Turret Tracking -------- */
        Pose pose = follower.getPose();

        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();

        // Angle from robot to goal (field-centric)
        double angleToGoal = Math.atan2(
                GOAL_Y - robotY,
                GOAL_X - robotX
        );

        // Desired turret angle in world frame (0–2π)
        double rawTarget = wrapAngle(angleToGoal - robotHeading);

        // Low-pass filter
        filteredTargetAngle = wrapAngle(
                filteredTargetAngle +
                        FILTER_ALPHA * shortestDelta(filteredTargetAngle, rawTarget)
        );

        // Current turret angle
        int turretTicks = turret.getCurrentPosition();
        double currentTurretAngle = wrapAngle(
                turretTicks / TICKS_PER_RAD
        );

        /* -------- Forbidden zone handling -------- */
        if (crossesForbidden(currentTurretAngle, filteredTargetAngle)) {
            // Force long way around (CW)
            filteredTargetAngle += 2.0 * Math.PI;
        }

        // Error (NO normalization)
        double error = filteredTargetAngle - currentTurretAngle;

        /* -------- Control -------- */
        if (Math.abs(error) < ANGLE_DEADBAND) {
            turret.setPower(0);
        } else {
            double power = (kP * error) + Math.signum(error) * kS;
            power = Range.clip(power, -MAX_POWER, MAX_POWER);

            // Encoder safety
            if ((turretTicks <= TURRET_MIN_TICKS && power < 0) ||
                    (turretTicks >= TURRET_MAX_TICKS && power > 0)) {
                power = 0;
            }

            turret.setPower(power);
        }

        /* -------- Telemetry -------- */
        telemetry.addData("Turret Angle (deg)", Math.toDegrees(currentTurretAngle));
        telemetry.addData("Target Angle (deg)", Math.toDegrees(filteredTargetAngle));
        telemetry.addData("Error (deg)", Math.toDegrees(error));
        telemetry.addData("Turret Ticks", turretTicks);
        telemetry.update();
    }





    /* ================= UTILS ================= */

    private double wrapAngle(double angle) {
        angle %= (2.0 * Math.PI);
        if (angle < 0) angle += 2.0 * Math.PI;
        return angle;
    }

    private double shortestDelta(double from, double to) {
        double delta = to - from;
        if (delta > Math.PI) delta -= 2.0 * Math.PI;
        if (delta < -Math.PI) delta += 2.0 * Math.PI;
        return delta;
    }

    private boolean crossesForbidden(double from, double to) {
        double delta = shortestDelta(from, to);
        double step = Math.signum(delta) * Math.toRadians(1);

        for (double a = from; Math.abs(a - from) < Math.abs(delta); a += step) {
            double w = wrapAngle(a);
            if (w > FORBIDDEN_START && w < FORBIDDEN_END) {
                return true;
            }
        }
        return false;
    }
}