package pedroPathing.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "WeCanDrive", group = "teleops")
public class WeCanDrive extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0,0,0);



    private void Joystick_Control() { //code for joystick control intake and lift
 //       RightLift.setPower(gamepad2.left_stick_y + gamepad2.left_stick_x);
 //       LeftLift.setPower(gamepad2.left_stick_y + gamepad2.left_stick_x);
 //       IntakeExtend.setPower(-gamepad2.right_stick_y + gamepad2.right_stick_x);
        telemetry.update();
    }

    private ElapsedTime     runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 537.6898 ;   // goBilda 5202 Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 19.2032 ;     // goBilda 5202 Gear ratio reduction
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // goBilda 5202 Wheel diameter
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.5;

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();


 //       telemetry.addData("Path0",  "Starting at %7d :%7d",SpinIntake.getCurrentPosition(),
  //              RightLift.getCurrentPosition(),LeftLift.getCurrentPosition());
        telemetry.update();


    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }
    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }
    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();

        if (gamepad2.dpad_left) { // eats sample

            Joystick_Control();
        }

        if (gamepad2.dpad_right) { // spits sample

            Joystick_Control();
        }

        if (gamepad2.right_bumper) {//brings in intake

            Joystick_Control();
        } else {

            Joystick_Control();
        }
        if (gamepad2.triangle) {//drops intake

            Joystick_Control();
        } else {

            Joystick_Control();
        }
        if (gamepad2.cross) {//brings in intake

            Joystick_Control();
        } else {

            Joystick_Control();
        }
        if (gamepad2.left_bumper) {

            Joystick_Control();
        }
        if (gamepad2.dpad_up) {

            Joystick_Control();
        }
        if (gamepad2.dpad_down) {

            Joystick_Control();
        }
        if (gamepad2.start) {
            //encoderLift(1, 0.5, 0.5, 2);
           // sleep(1);

        }
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}