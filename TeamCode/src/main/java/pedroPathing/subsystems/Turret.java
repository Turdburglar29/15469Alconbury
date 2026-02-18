package pedroPathing.subsystems;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class Turret {

    private DcMotorEx turretMotor;

    // ---- TUNING ----
    private static final double kP = 1.5;
    private static final double kD = 0.15;
    private static final double MAX_POWER = 1.0;

    // ---- ENCODER ----
    private static final double TICKS_PER_REV = 537.6; // change to your motor
    private static final double GEAR_RATIO = 3.0;      // turret gearing

    private double lastError = 0;
    private MathFunctions AngleUtils;

    public Turret(DcMotorEx motor) {
        turretMotor = motor;

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Convert encoder ticks → radians
    private double ticksToRadians(int ticks) {
        return (ticks / (TICKS_PER_REV * GEAR_RATIO)) * 2.0 * Math.PI;
    }

    // Call every loop
    public void update(
            Pose pose,
            double goalX,
            double goalY,
            double dt
    ) {
        // Robot pose
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading(); // radians

        // World angle to target
        double angleToTarget =
                Math.atan2(goalY - robotY, goalX - robotX);

        // Desired turret angle (relative to robot)
        double turretTarget =
                AngleUtils.normalizeAngle(angleToTarget - robotHeading);

        // Current turret angle
        double turretCurrent =
                ticksToRadians(turretMotor.getCurrentPosition());

        // Error
        double error =
                AngleUtils.normalizeAngle(turretTarget - turretCurrent);

        // PID
        double derivative = (error - lastError) / dt;
        double power = kP * error + kD * derivative;

        power = Range.clip(power, -MAX_POWER, MAX_POWER);
        turretMotor.setPower(power);

        lastError = error;
    }
}