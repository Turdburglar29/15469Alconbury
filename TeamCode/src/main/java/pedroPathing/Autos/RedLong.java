package pedroPathing.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import pedroPathing.subsystems.TurretController;

@Autonomous(name = "RedLong", group = "Auto")
public class RedLong extends OpMode {
    private ElapsedTime shotTimer = new ElapsedTime();
    private static final int bankVelocity = 1225;
    private DcMotor intake;
    private TurretController turretController;
    private DcMotorEx flywheel;
    private DcMotorEx flywheel2;
    private Servo BootKick;
    private Servo ballrelease;
    private RevBlinkinLedDriver led;
    private ElapsedTime slowDownTimer = new ElapsedTime();

    // flywheel and shooting fields
    private ElapsedTime flywheelEnableTimer = new ElapsedTime();
    private final double flywheelSpinUpDuration = 1.1;
    private final double flywheelSpinUpPower = 1;
    private static final double FW_KF = 0.0008;

    private static final double MANUAL_KP = 0.007;
    private static final double MANUAL_KI = 0.000001;
    private static final double MANUAL_KD = 0.00005;
    private double fwIntegrator = 0.0;
    private double fwLastError = 0.0;
    private double fwLastTime = -1.0;
    private final double OVERSHOOT_MARGIN = 30.0;
    private final double HOLDING_POWER_AFTER_OVERSHOOT = 0.25;

    // PID constants used to construct the legacy PIDController
    private static final double FW_KP = 0.006;
    private static final double FW_KI = 0.000001;
    private static final double FW_KD = 0.00005;

    // --- Flywheel PID / control fields (class scope) ---
    private PIDController flywheelPid;
    private double flywheelTargetVelocity = 0.0;
    private boolean flywheelPidEnabled = false;

    private ElapsedTime feedTimer = new ElapsedTime();
    private int shotsFired = 0;
    private final long feedIntervalMs = 400;
    private final long pauseBetweenFeedsMs = 600;
    private final long kickDurationMs = 1500;
    private boolean kickInProgress = false;
    private ElapsedTime kickTimer = new ElapsedTime();
    private final long safetyTimeoutMs = 6000;

    private enum ShootState { WAIT_FOR_READY, FEED_INTAKE, PAUSE_AFTER_FEED, BOOTKICK, DONE }
    private ShootState shootState = ShootState.WAIT_FOR_READY;
    private boolean feedTimerStarted = false;
    private boolean shootingSequenceStarted = false;

    private final double READY_LOWER_MARGIN = 0.0;
    private final double READY_UPPER_MARGIN = 30.0;
    private final boolean intakeDuringKick = true;

    private ElapsedTime runtime = new ElapsedTime();
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    //Start point-----------------------------------------------------------------------------------
    private final Pose startPose = new Pose(80, -6.19, Math.toRadians(90));
    //line 1 ScorePreload 1 ------------------------------------------------------------------------
    private final Pose scorePose = new Pose(75, 5, Math.toRadians(65));
    //Line 3 Pickup 1-------------------------------------------------------------------------------
    private final Pose pickup1Pose = new Pose(130, 22, Math.toRadians(0));
    private final Pose pickup1CP1 = new Pose(90, 20, Math.toRadians(0));
    private final Pose pickup1CP2 = new Pose(95, 20, Math.toRadians(0));
    //line 4 Score 1 -------------------------------------------------------------------------------
    private final Pose score1Pose = new Pose(75, 5, Math.toRadians(65));
    //line 6 Pickup  2 -----------------------------------------------------------------------------
    private final Pose pickup2Pose = new Pose(123, -2, Math.toRadians(-3));
    private final Pose pickup2CP1 = new Pose(90, -2, Math.toRadians(0));
    private final Pose pickup2CP2 = new Pose(95., -2, Math.toRadians(0));
    //line 7 Push Bar ------------------------------------------------------------------------------
    private final Pose pushBarPose = new Pose(16.5, 80, Math.toRadians(0));
    private final Pose pushBarCP1 = new Pose(25, 80, Math.toRadians(0));
    //line 8 Score  2 ------------------------------------------------------------------------------
    private final Pose score2Pose = new Pose(75, 5, Math.toRadians(65));
    private final Pose score2CP1 = new Pose(80,2, Math.toRadians(-5));
    private final Pose score2CP2 = new Pose(80, 2, Math.toRadians(50));
    //line 9 Pickup  3------------------------------------------------------------------------------
    //private final Pose pickup3Pose = new Pose(17, 115, Math.toRadians(180));
    //private final Pose pickup3CP1 = new Pose(50, 90, Math.toRadians(180));
    // private final Pose pickup3CP2 = new Pose(45, 110, Math.toRadians(180));
    //line 10 Score 3-------------------------------------------------------------------------------
    // private final Pose score3Pose = new Pose(32, 30, Math.toRadians(225));
    //  private final Pose score3CP1 = new Pose(25, 90, Math.toRadians(180));
    //  private final Pose score3CP2 = new Pose(30, 65, Math.toRadians(190));
    //line 10 Park----------------------------------------------------------------------------------
    private final Pose park = new Pose(80, 14.75, Math.toRadians(0));
    private Path scorePreload, Pickup1, Score1, Pickup2, PushBar, Score2, Pickup3, Score3, Park;

    // --- PIDController inner class (kept) ---
    private static class PIDController {
        private double kP, kI, kD;
        private double integral = 0.0;
        private double lastError = 0.0;
        private double lastTimestamp = -1.0;
        private double outputMin = 0.0;
        private double outputMax = 1.0;

        PIDController(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        void setOutputLimits(double min, double max) {
            this.outputMin = min;
            this.outputMax = max;
        }

        void reset() {
            integral = 0.0;
            lastError = 0.0;
            lastTimestamp = -1.0;
        }

        double update(double target, double measurement, double timestamp) {
            double error = target - measurement;
            if (lastTimestamp < 0) {
                lastTimestamp = timestamp;
                lastError = error;
                return clamp(kP * error);
            }
            double dt = Math.max(1e-6, timestamp - lastTimestamp);
            integral += error * dt;
            double derivative = (error - lastError) / dt;
            double output = kP * error + kI * integral + kD * derivative;
            lastError = error;
            lastTimestamp = timestamp;
            return clamp(output);
        }

        private double clamp(double v) {
            return Math.max(outputMin, Math.min(outputMax, v));
        }
    }

    public void buildPaths() {
        scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        Pickup1 = new Path(new BezierCurve(new Point(scorePose), new Point(pickup1CP1), new Point(pickup1CP2), new Point(pickup1Pose)));
        Pickup1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading());
        Score1 = new Path(new BezierCurve(new Point(pickup1Pose), new Point(score1Pose)));
        Score1.setLinearHeadingInterpolation(pickup1Pose.getHeading(), score1Pose.getHeading());
        Pickup2 = new Path(new BezierCurve(new Point(score1Pose), new Point(pickup2CP1), new Point(pickup2CP2), new Point(pickup2Pose)));
        Pickup2.setLinearHeadingInterpolation(score1Pose.getHeading(), pickup2Pose.getHeading());
        Score2 = new Path(new BezierCurve(new Point(pickup2Pose), new Point(score2CP1), new Point(score2CP2), new Point(score2Pose)));
        Score2.setLinearHeadingInterpolation(pickup2Pose.getHeading(), score2Pose.getHeading());
        Park = new Path(new BezierCurve(new Point(score2Pose), new Point(park)));
        Park.setLinearHeadingInterpolation(score2Pose.getHeading(), park.getHeading());
    }

    private void startShootingSequence() {
        flywheelTargetVelocity = bankVelocity;
        if (flywheelPid != null) flywheelPid.reset();
        flywheelPidEnabled = true;
        flywheelEnableTimer.reset();
        shotsFired = 0;
        feedTimer.reset();
        feedTimerStarted = false;
        shootState = ShootState.WAIT_FOR_READY;
        inFeedPause = false;
        feedPauseTimer.reset();
        kickInProgress = false;
        kickTimer.reset();
        shotTimer.reset();
        BootKick.setPosition(0.0);
        intake.setPower(0.0);
        ballrelease.setPosition(0.9);
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        shootingSequenceStarted = true;
        fwIntegrator = 0.0;
        fwLastError = 0.0;
        fwLastTime = -1.0;
    }

    private void stopFlyWheel(){
        flywheelTargetVelocity = 0.0;
        flywheelPidEnabled = false;
        if (flywheel != null) flywheel.setPower(0.0);
        if (flywheel2 != null) flywheel2.setPower(0.0);
    }

    private boolean inFeedPause = false;
    private ElapsedTime feedPauseTimer = new ElapsedTime();

    private boolean handleShootingCycle() {
        double current = 0.0;
        try { current = flywheel.getVelocity(); } catch (Exception e) { }
        boolean measuredWithinLower = current >= (flywheelTargetVelocity - READY_LOWER_MARGIN);
        boolean measuredTooHigh = current <= (flywheelTargetVelocity + READY_UPPER_MARGIN);
        boolean readyMeasured = measuredWithinLower && measuredTooHigh;

        if (shotTimer.milliseconds() >= safetyTimeoutMs) {
            intake.setPower(0.0);
            BootKick.setPosition(0.0);
            shootState = ShootState.DONE;
            shootingSequenceStarted = false;
            return true;
        }

        switch (shootState) {
            case WAIT_FOR_READY:
                if (readyMeasured) {
                    if (!feedTimerStarted) { feedTimer.reset(); feedTimerStarted = true; }
                    if (shotsFired < 2) shootState = ShootState.FEED_INTAKE;
                    else shootState = ShootState.BOOTKICK;
                } else {
                    intake.setPower(0.0);
                    feedTimerStarted = false;
                }
                break;
            case FEED_INTAKE:
                BootKick.setPosition(0.0);
                intake.setPower(1.0);
                if (feedTimer.milliseconds() >= feedIntervalMs) {
                    shotsFired++;
                    intake.setPower(0.0);
                    feedTimer.reset();
                    shootState = ShootState.PAUSE_AFTER_FEED;
                }
                break;
            case PAUSE_AFTER_FEED:
                intake.setPower(0.0);
                if (feedTimer.milliseconds() >= pauseBetweenFeedsMs) {
                    if (shotsFired < 2) shootState = ShootState.FEED_INTAKE;
                    else shootState = ShootState.BOOTKICK;
                }
                break;
            case BOOTKICK:
                intake.setPower(intakeDuringKick ? 1.0 : 0.0);
                if (!kickInProgress) {
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    BootKick.setPosition(0.6);
                    kickInProgress = true;
                    kickTimer.reset();
                } else {
                    if (kickTimer.milliseconds() >= kickDurationMs) {
                        BootKick.setPosition(0.0);
                        kickInProgress = false;
                        shotsFired++;
                        intake.setPower(0.0);
                        shootState = ShootState.DONE;
                        shootingSequenceStarted = false;
                        return true;
                    }
                }
                break;
            case DONE:
                intake.setPower(0.0);
                BootKick.setPosition(0.0);
                shootingSequenceStarted = false;
                return true;
        }

        telemetry.addData("shootState", shootState);
        telemetry.addData("measuredFW", current);
        telemetry.addData("readyMeasured", readyMeasured);
        telemetry.addData("shotsFired", shotsFired);
        telemetry.addData("feedTimer", feedTimer.milliseconds());
        telemetry.addData("kickInProgress", kickInProgress);
        telemetry.update();

        return false;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // ensure turret holds at zero while moving into position
                if (turretController != null) {
                    turretController.setHoldTargetTick(0);
                    turretController.setHoldGains(0.9, 0.01, 0.12);
                    turretController.setHoldAtZeroEnabled(true);
                }
                startShootingSequence();
                follower.setMaxPower(0.7);
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    if (!shootingSequenceStarted) startShootingSequence();
                    boolean done = handleShootingCycle();
                    if (done) {
                        BootKick.setPosition(0.6);
                        shootingSequenceStarted = false;
                        setPathState(2);
                    }
                }
                break;
            case 2:
                BootKick.setPosition(0.6);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                follower.followPath(Pickup1, true);
                stopFlyWheel();
                setPathState(3);
                break;
            case 3:
                ballrelease.setPosition(0.6);
                follower.setMaxPower(0.8);
                if(slowDownTimer.milliseconds() > 1200 ){
                    follower.setMaxPower(0.6);
                    BootKick.setPosition(0.0);
                    intake.setPower(1);
                }
                intake.setPower(1);
                if (!follower.isBusy()) setPathState(4);
                break;
            case 4:
                follower.followPath(Score1, true);
                follower.setMaxPower(0.6);
                intake.setPower(0);
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy()) {
                    if (!shootingSequenceStarted) startShootingSequence();
                    boolean done = handleShootingCycle();
                    if (done) {
                        slowDownTimer.reset();
                        shootingSequenceStarted = false;
                        setPathState(6);
                    }
                }
                break;
            case 6:
                follower.followPath(Pickup2, true);
                ballrelease.setPosition(0.45);
                BootKick.setPosition(0.0);
                intake.setPower(1);
                stopFlyWheel();
                setPathState(7);
                slowDownTimer.reset();
                break;
            case 7:
                if(slowDownTimer.milliseconds() > 1600) {
                    ballrelease.setPosition(0.45);
                    follower.setMaxPower(0.8);
                    setPathState(8);
                }
                break;
            case 8:
                setPathState(9);
                break;
            case 9:
                if (!follower.isBusy()) {
                    setPathState(10);
                    slowDownTimer.reset();
                }
                break;
            case 10:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                follower.setMaxPower(0.6);
                follower.followPath(Score2, true);
                if (slowDownTimer.milliseconds() > 3000) {
                    intake.setPower(0);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    if (!shootingSequenceStarted) startShootingSequence();
                    boolean done = handleShootingCycle();
                    if (done) {
                        shootingSequenceStarted = false;
                        setPathState(12);
                    }
                }
                slowDownTimer.reset();
                break;
            case 12:
                follower.followPath(Park, true);
                intake.setPower(0);
                stopFlyWheel();
                // keep turret holding at zero at end
                if (turretController != null) {
                    turretController.setHoldTargetTick(0);
                    turretController.setHoldAtZeroEnabled(true);
                }
                setPathState(17);
                slowDownTimer.reset();
                break;
            case 17:
                if (!follower.isBusy()) {
                    // ensure hold remains enabled and then stop
                    if (turretController != null) {
                        turretController.setHoldTargetTick(0);
                        turretController.setHoldAtZeroEnabled(true);
                    }
                    stop();
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        if (pathTimer != null) pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();

        // update turret controller first so it resists momentum immediately
        if (turretController != null) turretController.update();

        autonomousPathUpdate();

        // flywheel PID/manual control (unchanged)
        if (flywheelPidEnabled && flywheel != null) {
            double now = runtime.seconds();
            double measured = flywheel.getVelocity();
            double target = flywheelTargetVelocity;
            double power;
            if (flywheelEnableTimer.seconds() < flywheelSpinUpDuration) {
                power = flywheelSpinUpPower;
                fwIntegrator = 0.0;
                fwLastError = target - measured;
                fwLastTime = now;
            } else {
                double error = target - measured;
                double dt = (fwLastTime < 0) ? 0.02 : Math.max(1e-6, now - fwLastTime);
                double derivative = (fwLastTime < 0) ? 0.0 : (error - fwLastError) / dt;
                double provisionalPid = MANUAL_KP * error + MANUAL_KD * derivative + MANUAL_KI * fwIntegrator;
                double ff = FW_KF * target;
                double provisionalPower = provisionalPid + ff;
                boolean outputSaturatedHigh = provisionalPower > 1.0;
                boolean outputSaturatedLow = provisionalPower < 0.0;
                if (!outputSaturatedHigh && !outputSaturatedLow) fwIntegrator += error * dt;
                else {
                    if (outputSaturatedHigh && error < 0) fwIntegrator += error * dt;
                    if (outputSaturatedLow && error > 0) fwIntegrator += error * dt;
                }
                double pidOut = MANUAL_KP * error + MANUAL_KI * fwIntegrator + MANUAL_KD * derivative;
                double ffTerm = FW_KF * target;
                power = pidOut + ffTerm;
                if (measured > target + OVERSHOOT_MARGIN) {
                    power = Math.min(power, HOLDING_POWER_AFTER_OVERSHOOT);
                    fwIntegrator = 0.0;
                }
                power = Math.max(0.0, Math.min(1.0, power));
                fwLastError = error;
                fwLastTime = now;

            }
            flywheel.setPower(power);
            flywheel2.setPower(power);
        }

        // telemetry and LED
        try { telemetry.addData("intake Velocity", ((DcMotorEx) intake).getVelocity()); } catch (Exception e) {}
        double measuredFW = 0.0;
        try { measuredFW = flywheel.getVelocity(); } catch (Exception e) {}
        telemetry.addData("Flywheel Velocity", measuredFW);
        telemetry.addData("shotsFired", shotsFired);
        telemetry.addData("shootState", shootState);
        telemetry.addData("feedTimer ms", feedTimer.milliseconds());
        telemetry.addData("kickTimer ms", kickTimer.milliseconds());
        telemetry.addData("kickInProgress", kickInProgress);
        telemetry.update();
        telemetry.update();


    }

    @Override
    public void init() {
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        intake = hardwareMap.get(DcMotor.class, "intake");

        // instantiate TurretController and enable hold-at-zero immediately
        turretController = new TurretController(hardwareMap, "turret", follower);
        turretController.setHoldTargetTick(0);
        turretController.setHoldGains(0.9, 0.01, 0.12);
        turretController.setHoldAtZeroEnabled(true);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        ballrelease = hardwareMap.get(Servo.class,"ballrelease");
        BootKick = hardwareMap.get(Servo.class,"BootKick");
        led = hardwareMap.get(RevBlinkinLedDriver.class,"led");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheelPid = new PIDController(FW_KP, FW_KI, FW_KD);
        flywheelPid.setOutputLimits(0.0, 1.0);
        flywheelPid.reset();
        flywheelPidEnabled = false;
        flywheelEnableTimer.reset();

        feedTimer.reset();
        feedPauseTimer.reset();
        kickTimer.reset();
        BootKick.setPosition(0.0);

        fwIntegrator = 0.0;
        fwLastError = 0.0;
        fwLastTime = -1.0;
    }

    @Override
    public void start() {
        runtime.reset();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        // ensure turret remains held at zero when opmode stops
        if (turretController != null) {
            turretController.setHoldAtZeroEnabled(true);
        }
    }
}
