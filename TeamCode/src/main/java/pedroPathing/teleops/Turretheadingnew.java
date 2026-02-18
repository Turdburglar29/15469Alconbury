
package pedroPathing.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.subsystems.GoBildaPinpointDriver;

@TeleOp(name="Turretheadingnew")
public class Turretheadingnew extends LinearOpMode {

    // Device drivers
    GoBildaPinpointDriver pinpoint;
    DcMotorEx turret;

    // --- CONFIGURATION ---
    // 312 RPM Motor: 537.6 ticks per rev.
    // Adjust GEAR_RATIO if your motor connects to the turret via gears/belt (e.g., 2.0)
    final double GEAR_RATIO = 1.0;
    final double TICKS_PER_RADIAN = (537.6 * GEAR_RATIO) / (2 * Math.PI);

    // The angle you want the turret to ALWAYS point at (Field Centric)
    // 0 = Forward/Front Wall, 180 = Back Wall/Basket
    final double DESIRED_FIELD_ANGLE_DEG = 45;

    // Simple PID gain - start low and increase if the turret is "lazy"
    double Kp = 1.2;

    @Override
    public void runOpMode() {
        // 1. Hardware Mapping
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        // 2. Motor Setup
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 3. Pinpoint Calibration & Starting Angle
        // Replace with your actual [GoBilda Calibration Data](https://www.gobilda.com)
        pinpoint.setOffsets(-120.0, 0.0);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        // SET STARTING POSITION HERE: (X, Y, Angle)
        // This tells the robot it is starting at 0,0 facing 90 degrees
        pinpoint.updatePublishingPose(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 90));

        telemetry.addData("Status", "Pinpoint Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 4. Update Odometry
            pinpoint.update();
            Pose2D currentPose = pinpoint.getPosition();

            // 5. Calculate Relative Target
            // We take our fixed field goal and subtract the robot's current heading
            double targetRad = Math.toRadians(DESIRED_FIELD_ANGLE_DEG);
            double robotRad = currentPose.getHeading(AngleUnit.RADIANS);

            double relativeTargetRad = AngleUnit.normalizeRadians(targetRad - robotRad);

            // 6. Motor Control (Proportional)
            double targetTickPos = relativeTargetRad * TICKS_PER_RADIAN;
            double currentTickPos = turret.getCurrentPosition();

            double error = targetTickPos - currentTickPos;
            double motorPower = error * (Kp / TICKS_PER_RADIAN); // Normalized power

            // Safety cap for motor power
            motorPower = Math.max(-0.1, Math.min(0.1, motorPower));
            turret.setPower(motorPower);

            // 7. Feedback
            telemetry.addData("Robot Heading", Math.toDegrees(robotRad));
            telemetry.addData("Turret Error (Ticks)", error);
            telemetry.addData("Motor Power", motorPower);
            telemetry.update();
        }
    }
}