package pedroPathing.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
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
import pedroPathing.subsystems.TurretControllerRed;
import pedroPathing.subsystems.TurretControllerRedAuto;

@Autonomous(name = "RedLong", group = "Auto")

public class RedLong extends OpMode {
    private ElapsedTime shotTimer = new ElapsedTime();
    private static final int bankVelocity = 1225;
    private static final int farVelocity = 1800;
    private static final int idlVelocity = 800;
    private final double kF = 1.0 / 1200; //lower second number to increase speed up
    private final double kP = 0.0012                                                                            ; //increase if throughput is slow

    // === Distance thresholds (inches) ===
    private final double dNear = 20;
    private final double dFar = 140;
    private DcMotor intake;
    private TurretControllerRedAuto turretController;
    private DcMotorEx flywheel;
    private DcMotorEx flywheel2;
    private Servo BootKick;
    private Servo hood;
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
    private RedLong.PIDController flywheelPid;
    private double flywheelTargetVelocity = 0.0;
    private boolean flywheelPidEnabled = false;

    private ElapsedTime feedTimer = new ElapsedTime();
    private int shotsFired = 0;
    private final long feedIntervalMs = 400;
    private final long pauseBetweenFeedsMs = 600;
    private final long kickDurationMs = 1500;
    private boolean kickInProgress = false;
    private ElapsedTime kickTimer = new ElapsedTime();
    private final long safetyTimeoutMs = 7000;

    private enum ShootState {WAIT_FOR_READY, FEED_INTAKE, PAUSE_AFTER_FEED, BOOTKICK, DONE}

    private RedLong.ShootState shootState = RedLong.ShootState.WAIT_FOR_READY;
    private boolean feedTimerStarted = false;
    private boolean shootingSequenceStarted = false;

    private final double READY_LOWER_MARGIN = 0.0;
    private final double READY_UPPER_MARGIN = 30.0;
    private final boolean intakeDuringKick = true;

    private ElapsedTime runtime = new ElapsedTime();
    private Follower follower;
    private Timer pathTimer, opmodeTimer;


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


    // ---------------- PATH STATE ----------------
    private int pathState;

    // ---------------- POSES ----------------
    private final Pose startPose = new Pose(80, -6.19, Math.toRadians(90));
    private final Pose scorePose = new Pose(80, 6, Math.toRadians(90));

    private final Pose pickup1Pose = new Pose(123, 22, Math.toRadians(0));
    private final Pose pickup1CP1 = new Pose(90, 20, Math.toRadians(0));
    private final Pose pickup1CP2 = new Pose(95, 20, Math.toRadians(0));

    private final Pose score1Pose = new Pose(85, 6, Math.toRadians(0));

    private final Pose pickup2Pose = new Pose(126, -2, Math.toRadians(0));
    private final Pose pickup2CP1 = new Pose(90, -2, Math.toRadians(0));
    private final Pose pickup2CP2 = new Pose(95, -2, Math.toRadians(0));

    private final Pose score2Pose = new Pose(85, 6, Math.toRadians(0));
    private final Pose score2CP1 = new Pose(95, 2, Math.toRadians(0));
    private final Pose score2CP2 = new Pose(90, 2, Math.toRadians(0));

    private final Pose pickup3Pose = new Pose(126, -2, Math.toRadians(0));
    private final Pose pickup3CP1 = new Pose(90, -2, Math.toRadians(0));
    private final Pose pickup3CP2 = new Pose(95, -2, Math.toRadians(0));

    private final Pose score3Pose = new Pose(85, 2, Math.toRadians(0));
    private final Pose score3CP1 = new Pose(95, 2, Math.toRadians(0));
    private final Pose score3CP2 = new Pose(90, 2, Math.toRadians(0));

    private final Pose pickup4Pose = new Pose(126, 20, Math.toRadians(0));
    private final Pose pickup4CP1 = new Pose(90, 2, Math.toRadians(45));
    private final Pose pickup4CP2 = new Pose(95, 0, Math.toRadians(2));

    private final Pose score4Pose = new Pose(85, 4, Math.toRadians(0));
    private final Pose score4CP1 = new Pose(100, 8, Math.toRadians(40));
    private final Pose score4CP2 = new Pose(88, 4, Math.toRadians(20));

    private final Pose park = new Pose(90, 14.75, Math.toRadians(0));

    // ---------------- PATHS ----------------
    private Path scorePreload, Pickup1, Score1, Pickup2, Score2, Pickup3, Score3, Pickup4, Score4, Park;

    // ---------------- INIT ----------------
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
        turretController = new TurretControllerRedAuto(hardwareMap, "turret", follower);
        turretController.setHoldTargetTick(0);
        turretController.setHoldGains(0.9, 0.01, 0.12);
        turretController.setHoldAtZeroEnabled(true);
        turretController.setHeadingCcwPositive(false);
        turretController.setTickSoftLimitsEnabled(true);
        turretController.setTickLimits(-1650, 1050);
        turretController.setSoftMarginTicks(1);
        turretController.setSlowZoneTicks(15);

        //  follower.setStartingPose(new Pose(33, 120, Math.toRadians(270))); needs changed
        turretController.setMountOffsetRad(Math.toRadians(-165.6));

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        ballrelease = hardwareMap.get(Servo.class, "ballrelease");
        BootKick = hardwareMap.get(Servo.class, "BootKick");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        hood = hardwareMap.get(Servo.class, "hood");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);

        flywheelPid = new RedLong.PIDController(FW_KP, FW_KI, FW_KD);
        flywheelPid.setOutputLimits(0.0, 1.0);
        flywheelPid.reset();
        flywheelPidEnabled = false;
        flywheelEnableTimer.reset();

        feedTimer.reset();
        kickTimer.reset();
        BootKick.setPosition(0.0);

        fwIntegrator = 0.0;
        fwLastError = 0.0;
        fwLastTime = -1.0;
    }

    // ---------------- BUILD PATHS ----------------
    private void buildPaths() {
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

        Pickup3 = new Path(new BezierCurve(new Point(score3Pose), new Point(pickup3CP1), new Point(pickup3CP2), new Point(pickup3Pose)));
        Pickup3.setLinearHeadingInterpolation(score3Pose.getHeading(), pickup3Pose.getHeading());

        Score3 = new Path(new BezierCurve(new Point(pickup3Pose), new Point(score3CP1), new Point(score3CP2), new Point(score3Pose)));
        Score3.setLinearHeadingInterpolation(pickup3Pose.getHeading(), score3Pose.getHeading());

        Pickup4 = new Path(new BezierCurve(new Point(score4Pose), new Point(pickup4CP1), new Point(pickup4CP2), new Point(pickup4Pose)));
        Pickup4.setLinearHeadingInterpolation(score4Pose.getHeading(), pickup4Pose.getHeading());

        Score4 = new Path(new BezierCurve(new Point(pickup4Pose), new Point(score4CP1), new Point(score4CP2), new Point(score4Pose)));
        Score4.setLinearHeadingInterpolation(pickup4Pose.getHeading(), score4Pose.getHeading());

        Park = new Path(new BezierCurve(new Point(score2Pose), new Point(park)));
        Park.setLinearHeadingInterpolation(score2Pose.getHeading(), park.getHeading());
    }


    // ---------------- PATH STATE MACHINE ----------------
    private void setPathState(int s) {
        pathState = s;
        if (pathTimer != null) pathTimer.resetTimer();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                hood.setPosition(0.6);
                follower.setMaxPower(0.5);
                follower.followPath(scorePreload, true);

                setPathState(1);
                break;

            case 1:
                ((DcMotorEx) flywheel).setVelocity(bankVelocity);
                ((DcMotorEx) flywheel2).setVelocity(bankVelocity);

                if ((!follower.isBusy()) && ((DcMotorEx) flywheel2).getVelocity() >= -bankVelocity - 5 && (shotTimer.milliseconds() > 3000)) {//starts shooter
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    intake.setPower(1);
                }

                if(shotTimer.milliseconds() > 5000) {
                    setPathState(2);
                    ((DcMotorEx) flywheel).setVelocity(idlVelocity);
                    ((DcMotorEx) flywheel2).setVelocity(idlVelocity);
                }
                break;

            case 2:


                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                follower.followPath(Pickup1, true);

                setPathState(3);
                slowDownTimer.reset();
                break;

            case 3:
                ballrelease.setPosition(0.6);
                follower.setMaxPower(1.0);
                if (slowDownTimer.milliseconds() > 1200) {
                    follower.setMaxPower(0.8);
                    intake.setPower(1.0);
                }
                intake.setPower(1.0);
                if (!follower.isBusy()) setPathState(4);
                break;

            case 4:
                follower.followPath(Score1, true);
                follower.setMaxPower(0.9);
                intake.setPower(0.0);
                setPathState(5);
                break;

            case 5:
                if (!follower.isBusy()) {
                    //insert shot code here------------------------------------------------------
                    ((DcMotorEx) flywheel).setVelocity(bankVelocity);
                    ((DcMotorEx) flywheel2).setVelocity(bankVelocity);

                    if ((!follower.isBusy()) && ((DcMotorEx) flywheel2).getVelocity() >= -bankVelocity - 5 && (shotTimer.milliseconds() > 3000)) {//starts shooter
                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        intake.setPower(1);
                    }

                    if(shotTimer.milliseconds() > 5000) {
                        setPathState(6);
                        ((DcMotorEx) flywheel).setVelocity(idlVelocity);
                        ((DcMotorEx) flywheel2).setVelocity(idlVelocity);
                    }
                    slowDownTimer.reset();
                }
                break;

            case 6:

                follower.followPath(Pickup2, true);
                ballrelease.setPosition(0.45);
                BootKick.setPosition(0.0);
                intake.setPower(1.0);

                setPathState(7);
                slowDownTimer.reset();
                break;

            case 7:
                if (slowDownTimer.milliseconds() > 1200) {
                    ballrelease.setPosition(0.45);
                    follower.setMaxPower(1.0);
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
                follower.setMaxPower(0.9);
                follower.followPath(Score2, true);
                if (slowDownTimer.milliseconds() > 1200) {
                    intake.setPower(0.0);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    //insert shot code here------------------------------------------------------
                    ((DcMotorEx) flywheel).setVelocity(bankVelocity);
                    ((DcMotorEx) flywheel2).setVelocity(bankVelocity);

                    if ((!follower.isBusy()) && ((DcMotorEx) flywheel2).getVelocity() >= -bankVelocity - 5 && (shotTimer.milliseconds() > 3000)) {//starts shooter
                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        intake.setPower(1);
                    }

                    if(shotTimer.milliseconds() > 5000) {
                        setPathState(12);
                        ((DcMotorEx) flywheel).setVelocity(idlVelocity);
                        ((DcMotorEx) flywheel2).setVelocity(idlVelocity);
                    }
                    slowDownTimer.reset();
                }
                break;

            case 12:
                follower.followPath(Pickup3, true);
                ballrelease.setPosition(0.45);
                BootKick.setPosition(0.0);
                intake.setPower(1.0);
                setPathState(13);
                slowDownTimer.reset();
                break;

            case 13:
                if (slowDownTimer.milliseconds() > 1200) {
                    ballrelease.setPosition(0.45);
                    follower.setMaxPower(1.0);
                    setPathState(14);
                }
                break;

            case 14:
                setPathState(15);
                break;

            case 15:
                if (!follower.isBusy()) {
                    setPathState(16);
                    slowDownTimer.reset();
                }
                break;

            case 16:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                follower.setMaxPower(0.9);
                follower.followPath(Score3, true);
                if (slowDownTimer.milliseconds() > 1200) {
                    intake.setPower(0.0);
                    setPathState(17);
                }
                break;

            case 17:
                if (!follower.isBusy()) {
                    //insert shot code here------------------------------------------------------
                    ((DcMotorEx) flywheel).setVelocity(bankVelocity);
                    ((DcMotorEx) flywheel2).setVelocity(bankVelocity);

                    if ((!follower.isBusy()) && ((DcMotorEx) flywheel2).getVelocity() >= -bankVelocity - 5 && (shotTimer.milliseconds() > 3000)) {//starts shooter
                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        intake.setPower(1);
                    }

                    if(shotTimer.milliseconds() > 5000) {
                        setPathState(18);
                        ((DcMotorEx) flywheel).setVelocity(idlVelocity);
                        ((DcMotorEx) flywheel2).setVelocity(idlVelocity);
                    }
                    slowDownTimer.reset();
                }
                break;

            case 18:
                follower.followPath(Pickup4, true);
                ballrelease.setPosition(0.45);
                BootKick.setPosition(0.0);
                intake.setPower(1.0);
                setPathState(19);
                slowDownTimer.reset();
                break;

            case 19:
                if (slowDownTimer.milliseconds() > 1200) {
                    ballrelease.setPosition(0.45);
                    follower.setMaxPower(1.0);
                    setPathState(20);
                }
                break;

            case 20:
                setPathState(21);
                break;

            case 21:
                if (!follower.isBusy()) {
                    setPathState(22);
                    slowDownTimer.reset();
                }
                break;

            case 22:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                follower.setMaxPower(0.9);
                follower.followPath(Score4, true);
                if (slowDownTimer.milliseconds() > 1200) {
                    intake.setPower(0.0);
                    setPathState(23);
                }
                break;

            case 23:
                if (!follower.isBusy()) {
                    //insert shot code here------------------------------------------------------
                    ((DcMotorEx) flywheel).setVelocity(bankVelocity);
                    ((DcMotorEx) flywheel2).setVelocity(bankVelocity);

                    if ((!follower.isBusy()) && ((DcMotorEx) flywheel2).getVelocity() >= -bankVelocity - 5 && (shotTimer.milliseconds() > 3000)) {//starts shooter
                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        intake.setPower(1);
                    }

                    if(shotTimer.milliseconds() > 5000) {
                        setPathState(24);
                        ((DcMotorEx) flywheel).setVelocity(idlVelocity);
                        ((DcMotorEx) flywheel2).setVelocity(idlVelocity);
                    }
                    slowDownTimer.reset();
                }
                break;

            case 24:
                follower.followPath(Park, true);
                intake.setPower(0.0);
                setPathState(25);
                break;

            case 25:
                if (!follower.isBusy()) {
                    // end auto
                }
                break;
        }
    }

    // ---------------- LOOP ----------------
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
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        if (turretController != null) {
            follower.update();
            autonomousPathUpdate();

            Pose p = follower.getPose();
            double dx = TurretControllerRed.GOAL_X - p.getX();
            double dy = TurretControllerRed.GOAL_Y - p.getY();
            double distance = Math.hypot(dx, dy);

            double t = (distance - dNear) / (dFar - dNear);
            t = Math.max(0, Math.min(1, t));

            double targetVelocity = bankVelocity + t * (farVelocity - bankVelocity);

            double measuredVel = flywheel.getVelocity();
            double flyPower = computeFlywheelPower(targetVelocity, measuredVel);

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


            if (turretController != null) {
                turretController.update();
            }
            /* ---------------- TELEMETRY ---------------- */
            double fieldAngle = Math.atan2(TurretControllerRedAuto.GOAL_Y - p.getY(),
                    TurretControllerRedAuto.GOAL_X - p.getX());
            double desiredRad = TurretControllerRedAuto.wrapForTelemetry(
                    fieldAngle - (turretController.getHeadingSign() * p.getHeading()) - turretController.getMountOffsetRad()
            );
            int desiredTicks = (int) Math.round(desiredRad * (TurretControllerRedAuto.TURRET_TICKS_PER_REV / (2.0 * Math.PI)));

            int actualTicks = ((DcMotorEx) hardwareMap.get(DcMotorEx.class, "turret")).getCurrentPosition();


            // Feedback to Driver Hub
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());



            /* Telemetry Outputs of our Follower */
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

            /*/Tells you Flywheel Velocity */
            telemetry.addData("intake Velocity", ((DcMotorEx) intake).getVelocity());
            //telemetry.addData("Flywheel Velocity", ((DcMotorEx) shooter1).getVelocity());
            telemetry.addData("Flywheel Velocity", ((DcMotorEx) flywheel).getVelocity());
            telemetry.update();

            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
        }


    }
}
