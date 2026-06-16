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

@Autonomous(name = "RedLong", group = "Auto")

public class RedLong extends OpMode {
    private ElapsedTime shotTimer = new ElapsedTime();
    private static final int IDLVelocity = 500;
    private static final int bankVelocity = 1690;
    private static final int medVelocity = 750;
    private static final int farVelocity = 1000;
    private static final int maxVelocity = 2000;
    private static final int intakeVelocity = 1400;
    private DcMotor intake;
    private DcMotor kickstand;
    private DcMotor flywheel;
    private DcMotor flywheel2;

    private TurretControllerRed turret;

    private Servo sort1;
    private Servo hood;
    private Servo BootKick;
    private Servo ballrelease;
    private RevBlinkinLedDriver led;
    double hue;
    boolean lastCircle = false;
    private MathFunctions AngleUtils;
    private double turretEncoderRadians;
    private ElapsedTime slowDownTimer = new ElapsedTime();
    void setSafePower(DcMotor motor,double targetPower0){
        final double SLEW_RATE=0.2;
        double currentPower=motor.getPower();

        double desiredChange=currentPower;
        double limitedChange=Math.max(-SLEW_RATE,Math.min(desiredChange,SLEW_RATE));

        motor.setPower(currentPower += limitedChange);
    }
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime clipTimer = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 537.6898;   // goBilda 5202 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 19.2032;     // goBilda 5202 Gear ratio reduction
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // goBilda 5202 Wheel diameter
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    double startTime;

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

        follower = new Follower(hardwareMap);
        //follower.setStartingPose(new Pose(30, 75, Math.toRadians(180)));
        follower.setStartingPose(new Pose(33, 120, Math.toRadians(270)));

        /* === TURRET INIT === */
        turret = new TurretControllerRed(hardwareMap, "turret", follower);
        turret.setHeadingCcwPositive(false);
        turret.setTickSoftLimitsEnabled(true);
        turret.setTickLimits(-1650, 1050);
        turret.setSoftMarginTicks(1);
        turret.setSlowZoneTicks(15);

        turret.setMountOffsetRad(Math.toRadians(177));

        telemetry.update();
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");
        ballrelease = hardwareMap.get(Servo.class,"ballrelease");
        BootKick = hardwareMap.get(Servo.class,"BootKick");
        hood = hardwareMap.get(Servo.class,"hood");
        led = hardwareMap.get(RevBlinkinLedDriver.class,"led");
        // shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);
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

                follower.setMaxPower(0.5);
                follower.followPath(scorePreload, true);

                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {


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
                    setPathState(12);
                }
                slowDownTimer.reset();
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
                    setPathState(18);
                }
                slowDownTimer.reset();
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
                    setPathState(24);
                }
                slowDownTimer.reset();
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
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        if (turret != null) {
            turret.update();
        }

        /* ---------------- TELEMETRY ---------------- */
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
        int desiredTicks = (int) Math.round(desiredRad * (TurretController.TURRET_TICKS_PER_REV / (2.0 * Math.PI)));

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
