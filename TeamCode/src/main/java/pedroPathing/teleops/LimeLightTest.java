package pedroPathing.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "LimelightTest", group = "Examples")
public class LimeLightTest extends OpMode {
    private Follower follower;

    private Limelight3A LimeLight;
    private ElapsedTime shotTimer = new ElapsedTime();
    private static final int IDLVelocity = 200;
    private static final int bankVelocity = 500;
    private static final int medVelocity = 750;
    private static final int farVelocity = 1000;
    private static final int maxVelocity = 2000;
    private static final int intakeVelocity = 1400;
    private final Pose startPose = new Pose(0, 0, 0);
    public static DcMotor intake;
    public static DcMotor spinner1;
    private DcMotor shooter1;
    private DcMotor shooter2;
    private Servo releasespinner;
    private Servo sort1;
    private Servo sort2;
    private Servo BootKick;
    private Servo ballrelease;
    private RevBlinkinLedDriver leftled;
    private RevBlinkinLedDriver rightled;
    private NormalizedColorSensor test_color;
    double hue;
    boolean lastCircle = false;

void setSafePower(DcMotor motor,double targetPower0){
    final double SLEW_RATE=0.2;
    double currentPower=motor.getPower();

    double desiredChange=currentPower;
    double limitedChange=Math.max(-SLEW_RATE,Math.min(desiredChange,SLEW_RATE));

    motor.setPower(currentPower += limitedChange);
}
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 537.6898;   // goBilda 5202 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 19.2032;     // goBilda 5202 Gear ratio reduction
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // goBilda 5202 Wheel diameter
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);




    /**
     * This method is call once when init is played, it initializes the follower
     **/
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        telemetry.update();
        intake = hardwareMap.get(DcMotor.class, "intake");
        LimeLight = hardwareMap.get(Limelight3A.class,"LimeLight");
        LimeLight.pipelineSwitch(8);
        spinner1 = hardwareMap.get(DcMotor.class, "spinner1");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");
        ballrelease = hardwareMap.get(Servo.class,"ballrelease");
        BootKick = hardwareMap.get(Servo.class,"BootKick");
        test_color = hardwareMap.get(NormalizedColorSensor.class, "test_color");
        rightled = hardwareMap.get(RevBlinkinLedDriver.class,"rightled");
        leftled = hardwareMap.get(RevBlinkinLedDriver.class,"leftled");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);


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
    @Override
    public void start() {
        follower.startTeleopDrive();
    }
            /**
             * This is the main loop of the opmode and runs continuously after play
             **/
     public void loop() {
         follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
         follower.update();

         telemetry.addData("Light Detected", ((OpticalDistanceSensor) test_color).getLightDetected());
         NormalizedRGBA colors = test_color.getNormalizedColors();
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

         /* Telemetry Outputs of our Follower */
         telemetry.addData("X", follower.getPose().getX());
         telemetry.addData("Y", follower.getPose().getY());
         telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

         /*/Tells you Flywheel Velocity */
         telemetry.addData("Intake Velocity", ((DcMotorEx) intake).getVelocity());
         telemetry.addData("Flywheel Velocity", ((DcMotorEx) shooter1).getVelocity());
         telemetry.addData("Flywheel Velocity", ((DcMotorEx) shooter2).getVelocity());
         telemetry.update();

         rightled.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
         leftled.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);


         if (gamepad1.circle && !lastCircle) {
             shotTimer.reset();
         }
         lastCircle = gamepad1.circle;
         if (gamepad1.circle) {
             ballrelease.setPosition(0.2);
             ((DcMotorEx) shooter2).setVelocity(bankVelocity);
             if (((DcMotorEx) shooter2).getVelocity() >= bankVelocity - 10) {
                 rightled.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                 leftled.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                 intake.setPower(-1);
                 {

                 }
             } else {
                 intake.setPower(0);
             }
             if (shotTimer.milliseconds() > 3500) {
                 BootKick.setPosition(0.6);
             }
// ------------------------------------------------------------------
        } else if (gamepad1.cross) {
            ballrelease.setPosition(0.2);
            ((DcMotorEx) shooter2).setVelocity(medVelocity );
            if (((DcMotorEx) shooter2).getVelocity() >= medVelocity - 5) {
                rightled.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                leftled.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
               intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
//----------------------------------------------------------------------------
        } else if (gamepad1.triangle) {
                ballrelease.setPosition(0.2);
                ((DcMotorEx) shooter2).setVelocity(farVelocity);
                if (((DcMotorEx) shooter2).getVelocity() >= farVelocity - 5) {
                    intake.setPower(-1);
                    rightled.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    leftled.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                } else {
                    intake.setPower(0);
                }
 //----------------------------------------------------------------------------
        } else if (gamepad1.square) {
            ballrelease.setPosition(0.2);
            ((DcMotorEx) shooter2).setVelocity(bankVelocity);
            if (((DcMotorEx) shooter2).getVelocity() >= maxVelocity - 16) {
               intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        } else {
            ((DcMotorEx) shooter1).setVelocity(200);
            ((DcMotorEx) shooter2).setVelocity(200);
            shooter1.setPower(0);
            shooter2.setPower(0);
            intake.setPower(0);

//----------------------------------------------------------------------------
            if (gamepad1.dpad_right) {// intakes balls
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
            //----------------------------------------------------------------------------
            if (gamepad1.left_bumper) {// outtakes balls
                rightled.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                leftled.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                ballrelease.setPosition(0.4);
                intake.setPower(-1);
                BootKick.setPosition(0.37);
            } else {
                intake.setPower(0);
            }
            if (gamepad1.dpad_left) {
                rightled.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                leftled.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                BootKick.setPosition(0.6);
            }else {
                rightled.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                leftled.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                BootKick.setPosition(0.42);
                }

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
