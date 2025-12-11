package pedroPathing.teleops;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;


@TeleOp(name = "Only_Motor_Tester", group = "Examples")
public class Only_Motor_Tester extends OpMode {
    private Follower follower;

    private static final int bankVelocity = 1900;
    private static final int medVelocity = 1900;
    private static final int farVelocity = 2000;
    private static final int maxVelocity = 2450;
    private static final int intakeVelocity = 1400;
    public static DcMotor intake;
    public static DcMotor spinner1;
    private DcMotor shooter1;
    private DcMotor shooter2;

    private NormalizedColorSensor colorsensor;
    double hue;


    /**
     * This method is call once when init is played, it initializes the follower
     **/
    @Override
    public void init() {
        telemetry.update();
        intake = hardwareMap.get(DcMotor.class, "intake");
        spinner1 = hardwareMap.get(DcMotor.class, "spinner1");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        colorsensor = hardwareMap.get(NormalizedColorSensor.class, "colorsensor");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    /**
     * This method is called continuously after Init while waiting to be started.
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     **/
    /**
     * This is the main loop of the opmode and runs continuously after play
     **/
    @Override
    public void loop() {

        telemetry.addData("Light Detected", ((OpticalDistanceSensor) colorsensor).getLightDetected());
        NormalizedRGBA colors = colorsensor.getNormalizedColors();
        hue = JavaUtil.colorToHue(colors.toColor());

        //Determining the amount of red, green, and blue
        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);

        //Determining HSV and alpha
        telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
        telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colors.toColor()));
        telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colors.toColor()));
        telemetry.addData("Alpha", "%.3f", colors.alpha);
        /*/Tells you Flywheel Velocity */
        telemetry.addData("Intake Velocity", ((DcMotorEx) intake).getVelocity());
        telemetry.addData("Flywheel Velocity", ((DcMotorEx) shooter1).getVelocity());
        telemetry.addData("Flywheel Velocity", ((DcMotorEx) shooter2).getVelocity());
        telemetry.update();

        if (gamepad1.options) {
            shooter1.setPower(0.5);
        } else if (gamepad1.circle) {
            ((DcMotorEx) shooter2).setVelocity(bankVelocity - 1300);
            if (((DcMotorEx) shooter1).getVelocity() >= bankVelocity - 5) {

            } else {

            }
        } else if (gamepad1.cross) {
            ((DcMotorEx) shooter2).setVelocity(medVelocity - 640);
            if (((DcMotorEx) shooter1).getVelocity() >= medVelocity - 5) {

            } else {

            }
        } else if (gamepad1.triangle) {
                ((DcMotorEx) shooter2).setVelocity(farVelocity + 900);
                if (((DcMotorEx) shooter1).getVelocity() >= farVelocity - 5) {
                } else {

                }
        } else if (gamepad1.square) {
            ((DcMotorEx) shooter2).setVelocity(bankVelocity);
            if (((DcMotorEx) shooter1).getVelocity() >= maxVelocity - 16) {

            } else {

            }
        } else if (gamepad1.right_bumper) {
            ((DcMotorEx) intake).setVelocity(intakeVelocity -2400);
           if (((DcMotorEx) intake).getVelocity() >= intakeVelocity - 2000) {
            } else {

            }
        } else {
            ((DcMotorEx) shooter1).setVelocity(0);
            ((DcMotorEx) shooter2).setVelocity(0);
            shooter1.setPower(0);
            shooter2.setPower(0);
            intake.setPower(0);
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
            if (gamepad2.dpad_right) {// intakes balls
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
            //----------------------------------------------------------------------------
            if (gamepad1.left_bumper) {// outtakes balls
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
            if (hue < 350){
                telemetry.addData("Color", "Purple");


            }
            if (gamepad2.dpad_left) {
            }
            if (gamepad2.dpad_right) {
            }
            if (gamepad2.dpad_up) {
            }
            if (gamepad2.dpad_down) {
            }
            if (gamepad2.start) {
            }
        }
        /** We do not use this because everything automatically should disable **/
    }
}


