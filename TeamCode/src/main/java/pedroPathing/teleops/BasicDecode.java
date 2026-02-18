package pedroPathing.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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

import pedroPathing.subsystems.BlueTurretTracker;

@TeleOp(name = "BasicDecode", group = "Examples")
public class BasicDecode extends OpMode {
    private Follower follower;
    private double robotstartheading;

    private ElapsedTime shotTimer = new ElapsedTime();
    private static final int IDLVelocity = 350;
    private static final int bankVelocity = 500;
    private static final int medVelocity = 750;
    private static final int farVelocity = 1000;
    private static final int maxVelocity = 2000;
    private static final int intakeVelocity = 1400;
    private final Pose startPose = new Pose(0, 0, 0);
    private DcMotor intake;
    private DcMotor kickstand;
    private DcMotor turret;
    private DcMotor flywheel;
    private Servo releasespinner;
    private Servo sort1;
    private Servo sort2;
    private Servo BootKick;
    private Servo ballrelease;
    private RevBlinkinLedDriver led;
    double hue;
    double FTCDashboard;
    boolean lastCircle = false;
    private MathFunctions AngleUtils;
    private double turretEncoderRadians;

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
        kickstand = hardwareMap.get(DcMotor.class, "kickstand");
        turret = hardwareMap.get(DcMotor.class, "turret");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        ballrelease = hardwareMap.get(Servo.class,"ballrelease");
        BootKick = hardwareMap.get(Servo.class,"BootKick");
        led = hardwareMap.get(RevBlinkinLedDriver.class,"led");
        kickstand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    /**
     * This method is called continuously after Init while waiting to be started.
     **/
    @Override
    public void init_loop() {
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        //telemetry.addData("Light Detected", ((OpticalDistanceSensor) test_color).getLightDetected());
        //NormalizedRGBA colors = test_color.getNormalizedColors();
        // hue = JavaUtil.colorToHue(colors.toColor());

        //Determining the amount of red, green, and blue
        // telemetry.addData("Red", "%.3f", colors.red);
        // telemetry.addData("Green", "%.3f", colors.green);
        //  telemetry.addData("Blue", "%.3f", colors.blue);

        //Determining HSV and alpha
        //   telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
        //  telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colors.toColor()));
        //  telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colors.toColor()));
        //  telemetry.addData("Alpha", "%.3f", colors.alpha);

        /* Telemetry Outputs of our Follower */
        telemetry.addData("turret speed", ((DcMotorEx) turret).getPower());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /*/Tells you Flywheel Velocity */
        telemetry.addData("Intake Velocity", ((DcMotorEx) intake).getVelocity());
        telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
        telemetry.update();

        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);


        if (gamepad1.circle && !lastCircle) {
            shotTimer.reset();
        }
        lastCircle = gamepad1.circle;
        if (gamepad1.circle) {
            ballrelease.setPosition(0.3);
            ((DcMotorEx) flywheel).setVelocity(bankVelocity);
            if (((DcMotorEx) flywheel).getVelocity() >= bankVelocity - 10) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                intake.setPower(-1);
                {

                }
            } else {
                intake.setPower(0);
            }
            if (shotTimer.milliseconds() > 3100) {
                BootKick.setPosition(0.5);
            }
// ------------------------------------------------------------------
        } else if (gamepad1.cross) {
            ballrelease.setPosition(0);
            ((DcMotorEx) flywheel).setVelocity(medVelocity);
            if (((DcMotorEx) flywheel).getVelocity() >= medVelocity - 5) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
//----------------------------------------------------------------------------
        } else if (gamepad1.triangle) {
            ballrelease.setPosition(0.27);
            ((DcMotorEx) flywheel).setVelocity(farVelocity);
            if (((DcMotorEx) flywheel).getVelocity() >= farVelocity - 5) {
                intake.setPower(-1);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else {
                intake.setPower(0);
            }
            //----------------------------------------------------------------------------
        } else if (gamepad1.square) {
            ballrelease.setPosition(0.27);
            ((DcMotorEx) flywheel).setVelocity(bankVelocity);
            if (((DcMotorEx) flywheel).getVelocity() >= maxVelocity - 16) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        } else {
            ((DcMotorEx) flywheel).setVelocity(IDLVelocity);
            flywheel.setPower(0);
            intake.setPower(0);

//----------------------------------------------------------------------------
            if (gamepad1.dpad_right) {// intakes balls
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
            //----------------------------------------------------------------------------
            if (gamepad1.left_bumper) {// outtakes balls
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
                ballrelease.setPosition(0.45);
                intake.setPower(-1);
                BootKick.setPosition(0.8);
            } else {
                intake.setPower(0);
            }
            if (gamepad1.dpad_left) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                BootKick.setPosition(1);
            }else {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                BootKick.setPosition(0);
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
