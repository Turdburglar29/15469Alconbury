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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstantsTeleop;
@Disabled
@TeleOp(name = "lastfunctionalBlue", group = "BlueTeleOp")
public class AutoShotingBlue extends OpMode {

    private Follower follower;
    private DcMotorEx turret;
    private DcMotorEx flywheel;
    private DcMotor intake;
    private Servo ballrelease;
    private Servo BootKick;
    private RevBlinkinLedDriver led;
    private VoltageSensor voltageSensor;

    static final double TICKS_PER_REV = 145.1;
    static final double GEAR_RATIO = 70.0 / 14.0;
    static final double TICKS_PER_RAD = - (TICKS_PER_REV * GEAR_RATIO) / (2.0 * Math.PI);

    static final int TURRET_MIN_TICKS = - (int)(2 * Math.PI * Math.abs(TICKS_PER_RAD));
    static final int TURRET_MAX_TICKS = 0;

    static final double kP = 1.2;
    static final double kD = 0.004;
    static final double kF = 0.06;
    static final double MAX_POWER = 0.75;
    static final double ANGLE_DEADBAND = Math.toRadians(1.0);
    static final double FILTER_ALPHA = 0.15;

    static final double GOAL_X = 10.0;
    static final double GOAL_Y = 137.0;

    private double filteredTargetAngle = 0;
    private double lastTurretAngle = 0;
    private double turretStartOffset = 0;

    static final double MIN_DIST = 20;
    static final double MAX_DIST = 140;
    static final int MIN_VEL = 1200;
    static final int MAX_VEL = 2300;

    private ElapsedTime shotTimer = new ElapsedTime();
    private boolean turretLocked = false;
    private Object fakepose;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstantsTeleop.class);

        follower = new Follower(hardwareMap);
        follower.setPose(new Pose(30, 75, Math.toRadians(0))); // set known start
        // retrieve pose for init telemetry
        //Pose OtherstartPose = follower.setStartingPose();

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        ballrelease = hardwareMap.get(Servo.class, "ballrelease");
        BootKick = hardwareMap.get(Servo.class, "BootKick");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Turret starting offset in radians
        int currentTicks = turret.getCurrentPosition();
        turretStartOffset = currentTicks / TICKS_PER_RAD;

        // Compute required rotation to goal relative to starting pose
        /*double dx = GOAL_X - OtherstartPose.getX();
        double dy = GOAL_Y - OtherstartPose.getY();
        double angleToGoal = Math.atan2(dy, dx) - OtherstartPose.getHeading();

        double requiredTurn = wrapAngle(angleToGoal - turretStartOffset);

        telemetry.addData("startPose",OtherstartPose);
        telemetry.addData("Init Turret Angle (deg)", Math.toDegrees(turretStartOffset));
        telemetry.addData("Required Turn to Goal (deg)", Math.toDegrees(requiredTurn));
        telemetry.addData("Distance to Goal", Math.hypot(dx, dy));*/
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();

        // Compute absolute target for PID
        Pose startPose = follower.getPose();
        double dx = GOAL_X - startPose.getX();
        double dy = GOAL_Y - startPose.getY();
        double angleToGoal = Math.atan2(dy, dx) - startPose.getHeading();

        double currentTurretAngle = turret.getCurrentPosition() / TICKS_PER_RAD;
        filteredTargetAngle = wrapAngle(angleToGoal - currentTurretAngle);
        lastTurretAngle = currentTurretAngle;
    }

    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
        follower.update();

        Pose pose = follower.getPose();

        double dx = GOAL_X - pose.getX();
        double dy = GOAL_Y - pose.getY();
        double distance = Math.hypot(dx, dy);

        double robotVX = follower.getVelocity().getYComponent();
        double robotVY = follower.getVelocity().getXComponent();

        double leadX = GOAL_X - robotVX * 0.15;
        double leadY = GOAL_Y - robotVY * 0.15;

        double angleToGoal = Math.atan2(leadY - pose.getY(), leadX - pose.getX()) - pose.getHeading();
        double targetAngle = wrapAngle(angleToGoal);

        filteredTargetAngle = wrapAngle(filteredTargetAngle +
                FILTER_ALPHA * shortestDelta(filteredTargetAngle, targetAngle));

        double currentAngle = turret.getCurrentPosition() / TICKS_PER_RAD;
        double error = shortestDelta(currentAngle, filteredTargetAngle);
        double turretVelocity = currentAngle - lastTurretAngle;
        lastTurretAngle = currentAngle;

        int ticks = (int)(currentAngle * TICKS_PER_RAD);
        if (ticks <= TURRET_MIN_TICKS && error < 0) error = 0;
        if (ticks >= TURRET_MAX_TICKS && error > 0) error = 0;

        double power = kP * error - kD * turretVelocity;
        if (Math.abs(error) > ANGLE_DEADBAND) power += Math.signum(error) * kF;
        power *= 12.5 / voltageSensor.getVoltage();
        power = Range.clip(power, -MAX_POWER, MAX_POWER);
        if (Math.abs(error) < ANGLE_DEADBAND) power = 0;

        turret.setPower(power);

        turretLocked = Math.abs(error) < ANGLE_DEADBAND && Math.abs(turretVelocity) < 0.02;

        double scale = Range.clip((distance - MIN_DIST) / (MAX_DIST - MIN_DIST), 0, 1);
        int targetVel = (int)(MIN_VEL + scale * (MAX_VEL - MIN_VEL));
        flywheel.setVelocity(targetVel * (12.5 / voltageSensor.getVoltage()));
        boolean flywheelReady = Math.abs(flywheel.getVelocity() - targetVel) < 30;

        if (turretLocked && flywheelReady) led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        else if (flywheelReady) led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
        else led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

        if (gamepad1.circle && turretLocked && flywheelReady) {
            ballrelease.setPosition(0.3);
            if (shotTimer.milliseconds() > 250) BootKick.setPosition(0.5);
        } else {
            shotTimer.reset();
            BootKick.setPosition(0);
        }

        telemetry.addData("Turret Current Angle (deg)", Math.toDegrees(currentAngle));
        telemetry.addData("Filtered Target Angle (deg)", Math.toDegrees(filteredTargetAngle));
        telemetry.addData("Distance to Goal", distance);
        telemetry.addData("Turret Power", power);
        telemetry.update();
    }

    private double wrapAngle(double angle) {
        angle %= (2 * Math.PI);
        if (angle < 0) angle += 2 * Math.PI;
        return angle;
    }

    private double shortestDelta(double from, double to) {
        double delta = to - from;
        if (delta > Math.PI) delta -= 2 * Math.PI;
        if (delta < -Math.PI) delta += 2 * Math.PI;
        return delta;
    }
}