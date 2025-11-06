/* package pedroPathing.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.jetbrains.annotations.NotNull;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.subsystems.FTC_DECODE_SUBSYSTEM;

@Config
class ftc_decode_subsystem {

    public void getIntakestop(@NotNull HardwareMap hardwareMap) {
        intakestop = new FTC_DECODE_SUBSYSTEM(hardwareMap);
    }
    public void getIntakerun(@NotNull HardwareMap hardwareMap) {
        intakerun = new FTC_DECODE_SUBSYSTEM(hardwareMap);
    }
    public static FTC_DECODE_SUBSYSTEM intakerun;

    public static FTC_DECODE_SUBSYSTEM intakestop;

    @TeleOp(name = "DecodeTeleop", group = "teleops")
    public class DecodeTeleop extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 537.6898;   // goBilda 5202 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 19.2032;     // goBilda 5202 Gear ratio reduction
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // goBilda 5202 Wheel diameter
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.5;



    @Override
    public void init() {


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        follower.startTeleopDrive();
    }


    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();

        if (gamepad2.dpad_left) { // eats sample
            FTC_DECODE_SUBSYSTEM intakerun1 = intakerun;
        } else {
            FTC_DECODE_SUBSYSTEM intakestop1 = intakestop;

        }

        if (gamepad2.dpad_right) { // spits sample

        }

        if (gamepad2.right_bumper) {//brings in intake

        } else {

        }
        if (gamepad2.triangle) {//drops intake

        } else {

        }
        if (gamepad2.cross) {//brings in intake

        } else {

            if (gamepad2.left_bumper) {

            }
            if (gamepad2.dpad_up) {

            }
            if (gamepad2.dpad_down) {

            }
            if (gamepad2.start) {

            }
        }
    }
}

}


*/