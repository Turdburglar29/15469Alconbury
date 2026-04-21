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

@TeleOp(name = "AutoShotignBlueFar", group = "BlueTeleOp")
public class AutoShotingBlueFar extends OpMode {

    private Follower follower;
    private DcMotorEx flywheel;
    private DcMotorEx flywheel2;
    private DcMotor intake;
    private Servo ballrelease;
    private Servo BootKick;
    private RevBlinkinLedDriver led;

    private final ElapsedTime shotTimer = new ElapsedTime();
    private boolean lastCircle = false;

    // Velocity endpoints for interpolation
    private static final int bankVelocity = -575;
    private static final int farVelocity = -1300;

    private TurretController turret;

    // === PF constants ===
    private final double kF = 1.0 / 900; //lower second number to increase speed up
    private final double kP = 0.0015; //increase if throughput is slow

    // === Distance thresholds (inches) ===
    private final double dNear = 20;
    private final double dFar = 140;

    private double BootTimer = 3500;

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
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
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

    // === PF controller ===
    private double computeFlywheelPower(double targetVel, double measuredVel) {
        double ff = kF * targetVel;
        double error = targetVel - measuredVel;
        double pTerm = kP * error;
        double power = ff + pTerm;
        return Math.max(-1.0, Math.min(1.0, power));
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
        follower.update();

        if (gamepad1.options) {
            turret.calibrateMountOffsetToCurrentAim();
        }

        /* ---------------- DISTANCE‑BASED TARGET VELOCITY ---------------- */
        Pose p = follower.getPose();
        double dx = TurretController.GOAL_X - p.getX();
        double dy = TurretController.GOAL_Y - p.getY();
        double distance = Math.hypot(dx, dy);

        double t = (distance - dNear) / (dFar - dNear);
        t = Math.max(0, Math.min(1, t));

        double targetVelocity = bankVelocity + t * (farVelocity - bankVelocity);

        double measuredVel = flywheel.getVelocity();
        double flyPower = computeFlywheelPower(targetVelocity, measuredVel);

        /* ---------------- SHOOTER & INTAKE ---------------- */

        if (gamepad1.circle && !lastCircle) {
            shotTimer.reset();
        }
        lastCircle = gamepad1.circle;

        if (gamepad1.circle) {

            // === SHOOT FROM ANYWHERE ===
            ballrelease.setPosition(0.9);

            flywheel.setPower(flyPower);
            flywheel2.setPower(flyPower);

            if (Math.abs(measuredVel - targetVelocity) < 60) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                intake.setPower(-1);
            } else {
                intake.setPower(0);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
            }

            if (shotTimer.milliseconds() > 3500 &&
                    Math.abs(measuredVel - targetVelocity) < 60) {
                BootKick.setPosition(0.6);
            }

        } else {

            // === IDLE ===
            flywheel.setPower(0);
            flywheel2.setPower(0);
            intake.setPower(0);
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

            if (gamepad1.dpad_right) intake.setPower(1);

            if (gamepad1.left_bumper) {
                ballrelease.setPosition(0.46);
                intake.setPower(-1);
                BootKick.setPosition(0.8);
            }

            if (gamepad1.dpad_left) BootKick.setPosition(1);
            else BootKick.setPosition(0);

            if (gamepad1.dpad_down)
                turret.setMountOffsetRad(turret.getMountOffsetRad() + .03);

            if (gamepad1.dpad_up)
                turret.setMountOffsetRad(turret.getMountOffsetRad() - .03);
        }

        /* ---------------- TURRET UPDATE ---------------- */
        if (turret != null) turret.update();

        /* ---------------- TELEMETRY ---------------- */
        double fieldAngle = Math.atan2(TurretController.GOAL_Y - p.getY(),
                TurretController.GOAL_X - p.getX());
        double desiredRad = TurretController.wrapForTelemetry(
                fieldAngle - (turret.getHeadingSign() * p.getHeading()) - turret.getMountOffsetRad()
        );
        int desiredTicks = (int) Math.round(desiredRad * (TurretController.TURRET_TICKS_PER_REV / (2.0 * Math.PI)));

        int actualTicks = ((DcMotorEx) hardwareMap.get(DcMotorEx.class, "turret")).getCurrentPosition();

        telemetry.addData("Flywheel TargetVel", targetVelocity);
        telemetry.addData("Flywheel MeasuredVel", measuredVel);
        telemetry.addData("Flywheel Power", flyPower);
        telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("DesiredTicks", desiredTicks);
        telemetry.addData("ActualTicks", actualTicks);
        telemetry.update();
    }
}
