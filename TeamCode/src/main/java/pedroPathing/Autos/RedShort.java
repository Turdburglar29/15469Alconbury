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
import pedroPathing.subsystems.TurretControllerRedAuto;

@Autonomous(name = "RedShort", group = "Auto")

public class RedShort extends OpMode {
    private ElapsedTime shotTimer = new ElapsedTime();
    private static final int IDLVelocity = 500;
    private static final int bankVelocity = 1690;
    private static final int medVelocity = 750;
    private static final int farVelocity = 1000;
    private static final int maxVelocity = 2000;
    private static final int intakeVelocity = 1400;
    private DcMotor intake;
    private DcMotor flywheel;
    private DcMotor flywheel2;

    private TurretControllerRedAuto turret;

    private Servo hood;
    private Servo ballrelease;
    private RevBlinkinLedDriver led;

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
        private int pathState;
    //Start point-----------------------------------------------------------------------------------
        private final Pose startPose = new Pose(97, 120, Math.toRadians(270));
    //line 1 ScorePreload 1 ------------------------------------------------------------------------
        private final Pose scorePose = new Pose(80, 80, Math.toRadians(340));
    //Line 3 Pickup 1-------------------------------------------------------------------------------
    private final Pose pickup1Pose = new Pose(104, 70, Math.toRadians(360));
    private final Pose pickup1CP1 = new Pose(85, 68, Math.toRadians(360));
    private final Pose pickup1CP2 = new Pose(90, 73, Math.toRadians(360));
    //line 4 Score 1 -------------------------------------------------------------------------------
    private final Pose score1Pose = new Pose(50, 70, Math.toRadians(135));
    //line 6 Pickup  2 -----------------------------------------------------------------------------
    private final Pose pickup2Pose = new Pose(100, 44, Math.toRadians(360));
    private final Pose pickup2CP1 = new Pose(95, 39, Math.toRadians(360));
    private final Pose pickup2CP2 = new Pose(89, 42.5, Math.toRadians(360));
    //line 7 Push Bar ------------------------------------------------------------------------------
    private final Pose pushBarPose = new Pose(104, 58, Math.toRadians(360));
    private final Pose pushBarCP1 = new Pose(70, 50, Math.toRadians(360));
    private final Pose pushBarCP2 = new Pose(75, 50, Math.toRadians(360));
    private final Pose pushBarCP3 = new Pose(80, 50, Math.toRadians(360));
    //line 8 Score  2 ------------------------------------------------------------------------------
    private final Pose score2Pose = new Pose(80, 80, Math.toRadians(340));
    private final Pose score2CP1 = new Pose(90,45, Math.toRadians(200));
    private final Pose score2CP2 = new Pose(85, 50, Math.toRadians(200));
    //line 9 Pickup  3------------------------------------------------------------------------------
    private final Pose pickup3Pose = new Pose(108, 50.5, Math.toRadians(360));
    private final Pose pickup3CP1 = new Pose(77, 45, Math.toRadians(360));
    private final Pose pickup3CP2 = new Pose(87, 47.5, Math.toRadians(360));
    //line 10 Score 3-------------------------------------------------------------------------------
    // private final Pose score3Pose = new Pose(32, 30, Math.toRadians(225));
    //  private final Pose score3CP1 = new Pose(25, 90, Math.toRadians(180));
    //  private final Pose score3CP2 = new Pose(30, 65, Math.toRadians(190));
    //line 10 Park----------------------------------------------------------------------------------
    private final Pose park = new Pose(42, 55, Math.toRadians(176));
    //  private PathChain ;-------------------------------------------------------------------------
    private Path scorePreload,  Pickup1,Score1,Pickup2,PushBar,Score2,Pickup3,Score3,Park;
    //--------------------------------------------------------------------------------------------------
    public void buildPaths() {
//line 1 --------------------------------------------------------------------------------------------
        scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//Line 2 ---------------------------------------------------------------------------------------------------------------
        Pickup1 = new Path(new BezierCurve(new Point(scorePose), new Point(pickup2CP1), new Point(pickup2CP2), new Point(pickup2Pose)));
        Pickup1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading());
//line 3 ---------------------------------------------------------------------------------------------------------------
        Score1 = new Path(new BezierCurve(new Point(pushBarPose), new Point(score2CP1), new Point(score2CP2), new Point(score2Pose)));
        Score1.setLinearHeadingInterpolation(pushBarPose.getHeading(), score2Pose.getHeading());
//line 4 ----------------------------------------------------------------------------------------------------------------------------------
        Pickup2 = new Path(new BezierCurve(new Point(score2Pose), new Point(pickup1CP1), new Point(pickup1CP2), new Point(pickup1Pose)));
        Pickup2.setLinearHeadingInterpolation(score2Pose.getHeading(), pickup1Pose.getHeading());
//line 5 ----------------------------------------------------------------------------------------------------------------------------------
        PushBar = new Path(new BezierCurve(new Point(pickup3Pose), new Point(pushBarCP2), new Point(pushBarCP3), new Point(pushBarPose)));
        PushBar.setLinearHeadingInterpolation(pickup3Pose.getHeading(), pushBarPose.getHeading());
//line 6 ----------------------------------------------------------------------------------------------------------------------------------
        Score2 = new Path(new BezierCurve(new Point(pickup1Pose), new Point(score2CP1), new Point(score2CP2), new Point(score2Pose)));
        Score2.setLinearHeadingInterpolation(pickup1Pose.getHeading(), score2Pose.getHeading());
//line 7 ----------------------------------------------------------------------------------------------------------------------------------
        Pickup3 = new Path(new BezierCurve(new Point(scorePose), new Point(pickup3CP1), new Point(pickup3CP2), new Point(pickup3Pose)));
        Pickup3.setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading());
//line 8 ----------------------------------------------------------------------------------------------------------------------------------
        //Score3 = new Path(new BezierCurve(new Point(pickup3Pose), new Point(score3CP1), new Point(score3CP2), new Point(score3Pose)));
        // Score3.setLinearHeadingInterpolation(pickup3Pose.getHeading(), score3Pose.getHeading());
//line 9 ----------------------------------------------------------------------------------------------------------------------------------
        Park = new Path(new BezierCurve(new Point(score2Pose), new Point(park)));
        Park.setLinearHeadingInterpolation(score2Pose.getHeading(), park.getHeading());

    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                hood.setPosition(0.5);
                follower.setMaxPower(0.95);
                follower.followPath(scorePreload, true);
                setPathState(1);
                shotTimer.reset();
                break; // --------------------------------------Shoots balls1--------------------------------------------
            case 1:
                ((DcMotorEx) flywheel).setVelocity(bankVelocity);
                ((DcMotorEx) flywheel2).setVelocity(bankVelocity);

                if ((!follower.isBusy()) && ((DcMotorEx) flywheel2).getVelocity() >= -bankVelocity - 5 && (shotTimer.milliseconds() > 3000)) {//starts shooter
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                    intake.setPower(1);
                }

                if(shotTimer.milliseconds() > 4000) {
                    setPathState(2);
                }

                break; // --------------------------------------Picks up 1st line ---------------------------------------
            case 2:
                ballrelease.setPosition(0.9);
                follower.setMaxPower(0.95);
                follower.followPath(Pickup3, true);
                follower.setMaxPower(0.8);
                ((DcMotorEx) flywheel).setVelocity(0);//turns shooter off
                ((DcMotorEx) flywheel2).setVelocity(0);
                setPathState(3);
                break; // -------------------------------------------------------------------------------------------
            case 3:
                ballrelease.setPosition(0.9);
                follower.setMaxPower(0.8);
                if(slowDownTimer.milliseconds() > 1000 ){
                    follower.setMaxPower(0.7);
                    intake.setPower(1);
                }
                intake.setPower(1); //turns intake on
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break; // -------------------------------------Moves to Score--------------------------------
            case 4:
                follower.followPath(PushBar, true);
                follower.setMaxPower(0.5);
                setPathState(44);
                shotTimer.reset();
                break;
            case 44:
                if (!follower.isBusy() && (shotTimer.milliseconds() > 3000)) {
                    setPathState(5);
                }
                break;
            case 5:
                follower.setMaxPower(0.95);
                follower.followPath(Score1, true);
                ballrelease.setPosition(0.5);
                intake.setPower(0);
                flywheel.setPower(0);
                flywheel2.setPower(0);
                setPathState(6);//starts shooter
                ballrelease.setPosition(0.5);
                shotTimer.reset();


                break; // --------------------------------Shots Balls2-------------------------------------------
            case 6:
                if (!follower.isBusy()) {
                    //starts shooter
                    ((DcMotorEx) flywheel).setVelocity(bankVelocity);
                    ((DcMotorEx) flywheel2).setVelocity(bankVelocity);
                    if ((!follower.isBusy()) && ((DcMotorEx) flywheel2).getVelocity() >= -bankVelocity - 5 && (shotTimer.milliseconds() > 3000)) {
                        ballrelease.setPosition(0.5);
                        intake.setPower(1);
                    }
                    if(shotTimer.milliseconds() > 4000) {
                        setPathState(7);
                    }

                }
                slowDownTimer.reset();
                break; // -----------------------------------Picks up 2nd Line----------------------------------------
            case 7:
                follower.followPath(Pickup2, true);
                ballrelease.setPosition(0.9);
                intake.setPower(1);//turns shooter off
                setPathState(8);
                slowDownTimer.reset();
                break; // -------------------------------------------------------------------------------------------
            case 8:
                if(slowDownTimer.milliseconds() > 1600) {
                    ballrelease.setPosition(0.9);
                    follower.setMaxPower(.7);
                    intake.setPower(1);
                }
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                if (!follower.isBusy()) {
                    setPathState(9);
                    intake.setPower(0);
                }
                break; // ----------------------------------Hits Push Bar-----------------------------------
            case 9:
                follower.setMaxPower(0.95);
                /* follower.followPath(PushBar, true);*/
                setPathState(10);
                break; // -------------------------------------------------------------------------------------------
            case 10:

                if (!follower.isBusy()) {
                    intake.setPower(0);
                    setPathState(11);
                }
                break; // --------------------------------Moves to Score--------------------------------------------
            case 11:
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                intake.setPower(0);
                follower.setMaxPower(0.95);
                follower.followPath(Score2, true);
                setPathState(12);
                shotTimer.reset();
                break; // --------------------------------Shots Balls3--------------------------------------------
            case 12:
                ballrelease.setPosition(0.5);
                if (!follower.isBusy()) {
                    //starts shooter
                    ((DcMotorEx) flywheel).setVelocity(bankVelocity);
                    ((DcMotorEx) flywheel2).setVelocity(bankVelocity);
                    if ((!follower.isBusy()) && ((DcMotorEx) flywheel2).getVelocity() >= -bankVelocity - 5 && (shotTimer.milliseconds() > 3000)) {
                        intake.setPower(1);
                    }
                }
                if(shotTimer.milliseconds() > 4000) {
                    setPathState(17);
                }
                slowDownTimer.reset();
                break; // -------------------------------Picks up 3rd Line--------------------------------------------
            /*case 13:
                follower.followPath(Pickup3, true);
                intake.setPower(1);
                //turns shooter off
                setPathState(14);
                slowDownTimer.reset();
                break; // -------------------------------------------------------------------------------------------
            case 14:
                if(slowDownTimer.milliseconds() > 2000) {
                    follower.setMaxPower(1);
                    ballrelease.setPosition(0.9);
                    intake.setPower(1);
                }
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    setPathState(15);
                }
                break; // -------------------------------------------------------------------------------------------
            case 15:
                follower.setMaxPower(0.95);
                follower.followPath(Score2, true);
                setPathState(16);
                shotTimer.reset();
                break; // -------------------------------------------------------------------------------------------
            case 16:
                if (!follower.isBusy()) {
                    ballrelease.setPosition(0.5);
                    ((DcMotorEx) flywheel).setVelocity(bankVelocity);
                    ((DcMotorEx) flywheel2).setVelocity(bankVelocity);
                    if ((!follower.isBusy()) && ((DcMotorEx) flywheel2).getVelocity() >= -bankVelocity - 5 && (shotTimer.milliseconds() > 3000)) {

                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        intake.setPower(1);

                    }
                }
                if(shotTimer.milliseconds() > 4000) {
                    setPathState(17);
                }
                break; // -------------------------------------------------------------------------------------------
            case 17:
                follower.followPath(Park, true);
                setPathState(18);
                break; ?*/// -------------------------------------------------------------------------------------------
            case 18:
                if (!follower.isBusy()) {
                    stop();
                }
        }
    }
        public void setPathState(int pState) {
            pathState = pState;
            pathTimer.resetTimer();
        }

        @Override
        public void loop() {

            // These loop the movements of the robot
            follower.update();
            autonomousPathUpdate();

            if (turret != null) {
                turret.update();
            }
            /* ---------------- TELEMETRY ---------------- */
            Pose p = follower.getPose();
            double fieldAngle = Math.atan2(TurretControllerRedAuto.GOAL_Y - p.getY(),
                    TurretControllerRedAuto.GOAL_X - p.getX());
            double desiredRad = TurretControllerRedAuto.wrapForTelemetry(
                    fieldAngle - (turret.getHeadingSign() * p.getHeading()) - turret.getMountOffsetRad()
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
            follower.setStartingPose(new Pose(97, 120, Math.toRadians(270)));

            /* === TURRET INIT === */
            turret = new TurretControllerRedAuto(hardwareMap, "turret", follower);
            turret.setHeadingCcwPositive(false);
            turret.setTickSoftLimitsEnabled(true);
            turret.setTickLimits(-1650, 1050);
            turret.setSoftMarginTicks(1);
            turret.setSlowZoneTicks(15);

            turret.setMountOffsetRad(Math.toRadians(-177));

            telemetry.update();
            intake = hardwareMap.get(DcMotor.class, "intake");
            flywheel = hardwareMap.get(DcMotor.class, "flywheel");
            flywheel2 = hardwareMap.get(DcMotor.class, "flywheel2");
            ballrelease = hardwareMap.get(Servo.class,"ballrelease");
            hood = hardwareMap.get(Servo.class,"hood");
            led = hardwareMap.get(RevBlinkinLedDriver.class,"led");

            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
            flywheel2.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        @Override
        public void start() {
            opmodeTimer.resetTimer();
            setPathState(0);
        }

        @Override
        public void stop() {
        }
    }

