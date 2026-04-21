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

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstantsTeleop;
import pedroPathing.subsystems.TurretController;
@Disabled

@TeleOp(name = "BlueTeleOp2", group = "BlueTeleOp")
public class BlueTeleOp2 extends OpMode {

    private Follower follower;
    private DcMotorEx flywheel;
    private DcMotorEx flywheel2;
    private DcMotor intake;
    private Servo ballrelease;
    private Servo BootKick;
    private RevBlinkinLedDriver led;

    private final ElapsedTime shotTimer = new ElapsedTime();
    private boolean lastCircle = false;
    private boolean lastRightBumper = false;

    /*private static final int IDLVelocity = 500;
    private static final int bankVelocity = 1000;
    private static final int medVelocity = 1400;
    private static final int farVelocity = 2200;
    private static final int maxVelocity = 2000;
     */

    private TurretController turret;

    /*================== FLYWHEEL PDF CONTROL ==================== */
    double curTargetVelocity;
    double lowvelocity = 1000;
    double middlevelocity = 1800;
    double P = 0;
    double F = 0;
    double[] stepSizes = {10.0,1.0,0.1,00.1,000.1};

    int stepIndex = 1;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstantsTeleop.class);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(57, 75, Math.toRadians(180)));

        /* === TURRET INIT === */
        turret = new TurretController(hardwareMap, "turret", follower);
        turret.setHeadingCcwPositive(false);
        turret.setTickSoftLimitsEnabled(true);
        turret.setTickLimits(-1650, 1050);
        turret.setSoftMarginTicks(1);
        turret.setSlowZoneTicks(15);

        turret.setMountOffsetRad(Math.toRadians(-177));

        /* === SHOOTER INIT === */
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        ballrelease = hardwareMap.get(Servo.class, "ballrelease");
        BootKick = hardwareMap.get(Servo.class, "BootKick");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        telemetry.addLine("Init OK");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        /* ---------------- DRIVE ---------------- */
        follower.setTeleOpMovementVectors(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
        follower.update(); // keep pose fresh


        // One-button calibration: manually aim at the real goal, then press OPTIONS once
        if (gamepad1.circleWasPressed()){
            if (curTargetVelocity == middlevelocity) {
                curTargetVelocity = lowvelocity;
            }else{curTargetVelocity = middlevelocity; }

        }
        /*if (gamepad1.bWasPressed()) {
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

         */



        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flywheel.setVelocity(curTargetVelocity);
        flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        flywheel2.setVelocity(curTargetVelocity);
        /*if (gamepad1.rightBumperWasPressed()){
            shotTimer.reset();
            ballrelease.setPosition(0.3);
            stepIndex =(stepIndex = 10) % stepSizes.length;
            P -= stepSizes[stepIndex];
            F -= stepSizes[stepIndex];
            if (shotTimer.milliseconds() > 1100){
                intake.setPower(1);



        }
        if (gamepad1.rightBumperWasReleased()){
            stepIndex =(stepIndex = 10) % stepSizes.length;
            P += stepSizes[stepIndex];
            F += stepSizes[stepIndex];

        }

         */


        if(gamepad1.triangleWasPressed()){
            stepIndex =(stepIndex = 10) % stepSizes.length;
            P -= stepSizes[stepIndex];
            F -= stepSizes[stepIndex];
            if(((flywheel.getVelocity() / flywheel2.getVelocity()) <= curTargetVelocity - 20) && (flywheel.getVelocity() / flywheel2.getVelocity()) >= curTargetVelocity - 20) {
                intake.setPower(-1);
            }
        }


        double curVelocity = flywheel.getVelocity();
        double error2 = curTargetVelocity - curVelocity;

        //if (gamepad1.options) {
        //    turret.calibrateMountOffsetToCurrentAim();
        //}

        /* ---------------- SHOOTER & INTAKE ---------------- */
        if (gamepad1.circle && !lastCircle) {
            shotTimer.reset();
        }
        lastCircle = gamepad1.circle;

        if (gamepad1.cross) {
            /*ballrelease.setPosition(0.3);
            ((DcMotorEx) flywheel).setVelocity(bankVelocity);
            if (((DcMotorEx) flywheel).getVelocity() >= bankVelocity - 10) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                intake.setPower(-1);
            } else {
                intake.setPower(0);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
            }
            if (shotTimer.milliseconds() > 2500) {
                BootKick.setPosition(0.5);
            }

             */

        } else if (gamepad1.cross) {
            /*shotTimer.reset();
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

             */

        } else if (gamepad1.triangle) {
            /*ballrelease.setPosition(0.27);
            ((DcMotorEx) flywheel).setVelocity(farVelocity);
            if (((DcMotorEx) flywheel).getVelocity() >= farVelocity - 5) {
                intake.setPower(-1);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else {
                intake.setPower(0);
            }

             */

        } else if (gamepad1.square) {
            /*ballrelease.setPosition(0.27);
            ((DcMotorEx) flywheel).setVelocity(bankVelocity);
            if (((DcMotorEx) flywheel).getVelocity() >= maxVelocity - 16) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

             */

        } else {
            /*((DcMotorEx) flywheel).setVelocity(IDLVelocity);
            flywheel.setPower(0);
            intake.setPower(0);

             */



            if (gamepad1.dpad_right) { // intakes balls
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.left_bumper) { // outtakes balls
                ballrelease.setPosition(0.45);
                intake.setPower(-1);
                BootKick.setPosition(0.8);
            } else {
                intake.setPower(0);
            }
            while (gamepad1.right_bumper) { // outtakes balls
                ballrelease.setPosition(0.3);
                intake.setPower(-1);
                intake.setPower(0);
                if (shotTimer.milliseconds() > 1500) {
                    BootKick.setPosition(0.5);
                }
            }

            if (gamepad1.dpad_left) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                BootKick.setPosition(1);
            } else {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                BootKick.setPosition(0);
            }
        }

        /* ---------------- TURRET UPDATE (ALWAYS) ---------------- */
        if (turret != null) {
            turret.update();
        }

        /* ---------------- TELEMETRY ---------------- */
        Pose p = follower.getPose();
        double fieldAngle = Math.atan2(TurretController.GOAL_Y - p.getY(),
                TurretController.GOAL_X - p.getX());
        double desiredRad = TurretController.wrapForTelemetry(
                fieldAngle - (turret.getHeadingSign() * p.getHeading()) - turret.getMountOffsetRad()
        );
        int desiredTicks = (int) Math.round(desiredRad * (TurretController.TURRET_TICKS_PER_REV / (2.0*Math.PI)));

        int actualTicks = ((DcMotorEx) hardwareMap.get(DcMotorEx.class, "turret")).getCurrentPosition();

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity","x.2f",curVelocity);
        telemetry.addData(" error2",error2);
        telemetry.addData("MountOffset(deg)", Math.toDegrees(turret.getMountOffsetRad()));
        telemetry.addData("DesiredTicks", desiredTicks);
        telemetry.addData("ActualTicks", actualTicks);
        telemetry.addData("ErrorTicks", (desiredTicks - actualTicks));
        telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("Goal", "(%.1f, %.1f)", TurretController.GOAL_X, TurretController.GOAL_Y);
        telemetry.update();
    }
}