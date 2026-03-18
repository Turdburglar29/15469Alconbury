package pedroPathing.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstantsTeleop;
@Disabled
@TeleOp(name = "RedTeleopPID", group = "RedTeleopPID")
public class RedTeleopPID extends OpMode {

    /* ================= HARDWARE ================= */
    private Follower follower;
    private DcMotor turret;
    private DcMotorEx flywheel;
    private DcMotor intake;
    private DcMotorEx flywheel2;
    private Servo ballrelease;
    private Servo BootKick;
    private RevBlinkinLedDriver led;
    private ElapsedTime ParkNowTimer = new ElapsedTime();
    /*================== FLYWHEEL PDF CONTROL ==================== */
    double curTargetVelocity;
    double lowvelocity = 1000;
    double middlevelocity = 1800;
    double P = 0;
    double F = 0;
    double[] stepSizes = {10.0,1.0,0.1,00.1,000.1};
    int stepIndex = 1;

    /* ================= TURRET CONSTANTS ================= */
    static final double TICKS_PER_REV = 145.6;
    static final double GEAR_RATIO = 70.0 / 14.0;
    static final double TICKS_PER_RAD = (TICKS_PER_REV * GEAR_RATIO) / (2.0 * Math.PI);

    static final int TURRET_MIN_TICKS = 0;
    static final double MAX_TURRET_RAD = 2.0 * Math.PI;
    static final int TURRET_MAX_TICKS = (int) (MAX_TURRET_RAD * TICKS_PER_RAD);

    static final double kP = 3;
    static final double kS = 0.0;
    static final double MAX_POWER = 0.3;
    static final double ANGLE_DEADBAND = Math.toRadians(1.0);
    static final double FILTER_ALPHA = 0.10;

    static final double GOAL_X = 140.0;
    static final double GOAL_Y = 132.5;

    private double filteredTargetAngle = 0.0;

    /* ================= SHOOTER CONSTANTS ================= */
    private ElapsedTime shotTimer = new ElapsedTime();
    private boolean lastCircle = false;
    private static final int IDLVelocity = 500;
    private static final int bankVelocity = 1000;
    private static final int medVelocity = 1400;
    private static final int farVelocity = 1700;
    private static final int maxVelocity = 2000;

    public RedTeleopPID() {
        curTargetVelocity = lowvelocity;
    }

    /* ================= INIT ================= */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstantsTeleop.class);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(120, 75, Math.toRadians(0)));

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        ballrelease = hardwareMap.get(Servo.class, "ballrelease");
        BootKick = hardwareMap.get(Servo.class, "BootKick");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        filteredTargetAngle = 0.0;
    }
    @Override
    public void loop() {
        /* ---------------- DRIVE ---------------- */
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
        follower.update();
        if (ParkNowTimer.milliseconds() >= 115000)
            gamepad1.rumble(2000);

        /* ---------------- TURRET AUTO AIM Variables ---------------- */
        Pose pose = follower.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeading = pose.getHeading();
        double angleToGoal = Math.atan2(GOAL_Y - robotY, GOAL_X - robotX);
        double rawTarget = wrapAngle(angleToGoal - robotHeading);
        filteredTargetAngle = wrapAngle(
                filteredTargetAngle + FILTER_ALPHA * shortestDelta(filteredTargetAngle, rawTarget));
        int turretTicks = turret.getCurrentPosition();
        double currentTurretAngle = wrapAngle(turretTicks / TICKS_PER_RAD);
        double error = shortestDelta(currentTurretAngle, filteredTargetAngle);
        /* ---------------- TURRET AUTO AIM Math to set and execute target location ---------------- */
        if (Math.abs(error) < ANGLE_DEADBAND) {
            turret.setPower(0);
        } else {
            double power = kP * error + Math.signum(error) * kS;
            power = Range.clip(power, -MAX_POWER, MAX_POWER);
            if ((turretTicks <= TURRET_MIN_TICKS && power < 0) ||
                    (turretTicks >= TURRET_MAX_TICKS && power > 0)) {
                power = 0;
            }
            turret.setPower(power);
        }

        /* ---------------- SHOOTER & INTAKE ---------------- */
        if (gamepad1.rightBumperWasPressed()){
            if (curTargetVelocity == middlevelocity) {
                curTargetVelocity = lowvelocity;
            }else{curTargetVelocity = middlevelocity; }

        }
        if (gamepad1.bWasPressed()) {
            stepIndex =(stepIndex = 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flywheel.setVelocity(curTargetVelocity);

        double curVelocity = flywheel.getVelocity();
        double error2 = curTargetVelocity - curVelocity;

        /* ---------------- TELEMETRY ---------------- */
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity","x.2f",curVelocity);
        telemetry.addData(" error2",error2);
        telemetry.addData("Turret Angle (deg)", Math.toDegrees(currentTurretAngle));
        telemetry.addData("Target Angle (deg)", Math.toDegrees(filteredTargetAngle));
        telemetry.addData("Error (deg)", Math.toDegrees(error));
        telemetry.addData("Turret Ticks", turretTicks);
        telemetry.addData("X", pose.getX());
        telemetry.addData("Y", pose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(robotHeading));
        telemetry.addLine("init complete");
        telemetry.update();

        if (gamepad1.rightBumperWasPressed()){
            shotTimer.reset();
            ballrelease.setPosition(0.3);
            stepIndex =(stepIndex = 10) % stepSizes.length;
            P -= stepSizes[stepIndex];
            F -= stepSizes[stepIndex];
            if (shotTimer.milliseconds() > 1100){
                intake.setPower(1);
            }

            }
        if (gamepad1.rightBumperWasReleased()){
            stepIndex =(stepIndex = 10) % stepSizes.length;
            P += stepSizes[stepIndex];
            F += stepSizes[stepIndex];

        }
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
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
                intake.setPower(0);
            }
            if (shotTimer.milliseconds() > 2500) {
                BootKick.setPosition(0.5);
            }
// ------------------------------------------------------------------
        } else if (gamepad1.cross) {
            shotTimer.reset();
            ballrelease.setPosition(0);
            ((DcMotorEx) flywheel).setVelocity(medVelocity);
            if (((DcMotorEx) flywheel).getVelocity() >= medVelocity - 5) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

        if (shotTimer.milliseconds() > 2500) {
            BootKick.setPosition(0.5);
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
    }
