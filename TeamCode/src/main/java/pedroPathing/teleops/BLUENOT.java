package pedroPathing.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstantsTeleop;
import pedroPathing.subsystems.TurretController;

@TeleOp(name = "BLUENOT", group = "BlueTeleOp")
public class BLUENOT extends OpMode {

    private Follower follower;
    private DcMotor flywheel;
    private DcMotor flywheel2;
    private DcMotor intake;
    private Servo ballrelease;
    private Servo BootKick;
    private RevBlinkinLedDriver led;

    private final ElapsedTime shotTimer = new ElapsedTime();
    private boolean lastCircle = false;

    private static final int IDLVelocity = 500;
    private static final int bankVelocity = 1000;
    private static final int medVelocity = 1400;
    private static final int farVelocity = 2200;
    private static final int maxVelocity = 2000;

    private TurretController turret;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstantsTeleop.class);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(30, 75, Math.toRadians(180)));

        /* === TURRET INIT === */
        turret = new TurretController(hardwareMap, "turret", follower);
        turret.setHeadingCcwPositive(false);
        turret.setTickSoftLimitsEnabled(true);
        turret.setTickLimits(-365,340 );
        turret.setSoftMarginTicks(3);
        turret.setSlowZoneTicks(50);

        turret.setMountOffsetRad(Math.toRadians(-173));

        /* === SHOOTER INIT === */
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

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
        if (gamepad1.options) {
            turret.calibrateMountOffsetToCurrentAim();
        }

        /* ---------------- SHOOTER & INTAKE ---------------- */
        if (gamepad1.circle && !lastCircle) {
            shotTimer.reset();
        }
        lastCircle = gamepad1.circle;

        if (gamepad1.circle) {
            ballrelease.setPosition(0.3);
            ((DcMotorEx) flywheel).setVelocity(bankVelocity);
            ((DcMotorEx) flywheel2).setVelocity(bankVelocity);
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

        } else if (gamepad1.cross) {
            shotTimer.reset();
            ballrelease.setPosition(0);
            ((DcMotorEx) flywheel).setVelocity(medVelocity);
            ((DcMotorEx) flywheel2).setVelocity(medVelocity);
            if (((DcMotorEx) flywheel).getVelocity() >= medVelocity - 5) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
            if (shotTimer.milliseconds() > 2500) {
                BootKick.setPosition(0.5);
            }

        } else if (gamepad1.triangle) {
            ballrelease.setPosition(0.27);
            ((DcMotorEx) flywheel).setVelocity(farVelocity);
            ((DcMotorEx) flywheel2).setVelocity(farVelocity);
            if (((DcMotorEx) flywheel).getVelocity() >= farVelocity - 920) {
                intake.setPower(-1);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else {
                intake.setPower(0);
            }

        } else if (gamepad1.square) {
            ballrelease.setPosition(0.27);
            ((DcMotorEx) flywheel).setVelocity(bankVelocity);
            ((DcMotorEx) flywheel2).setVelocity(bankVelocity);
            if (((DcMotorEx) flywheel).getVelocity() >= maxVelocity - 16) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

        } else {
            ((DcMotorEx) flywheel).setVelocity(IDLVelocity);
            ((DcMotorEx) flywheel2).setVelocity(IDLVelocity);
            flywheel.setPower(0);
            flywheel2.setPower(0);
            intake.setPower(0);

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

        telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
        telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel2).getVelocity());
        telemetry.addData("MountOffset(deg)", Math.toDegrees(turret.getMountOffsetRad()));
        telemetry.addData("DesiredTicks", desiredTicks);
        telemetry.addData("ActualTicks", actualTicks);
        telemetry.addData("ErrorTicks", (desiredTicks - actualTicks));
        telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("Goal", "(%.1f, %.1f)", TurretController.GOAL_X, TurretController.GOAL_Y);
        telemetry.update();
    }
}