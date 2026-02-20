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

@Autonomous(name = "BlueShort", group = "Auto")

    public class BlueShort extends OpMode {
    private ElapsedTime shotTimer = new ElapsedTime();
    private static final int IDLVelocity = 500;
    private static final int bankVelocity = 1000;
    private static final int medVelocity = 750;
    private static final int farVelocity = 1000;
    private static final int maxVelocity = 2000;
    private static final int intakeVelocity = 1400;
    private DcMotor intake;
    private DcMotor kickstand;
    private DcMotor turret;
    private DcMotor flywheel;
    private Servo sort1;
    private Servo sort2;
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
        private int pathState;
    //Start point-----------------------------------------------------------------------------------
        private final Pose startPose = new Pose(9, 100.9, Math.toRadians(135));
    //line 1 ScorePreload 1 ------------------------------------------------------------------------
        private final Pose scorePose = new Pose(21.68, 96.6776, Math.toRadians(135));
    //Line 3 Pickup 1-------------------------------------------------------------------------------
        private final Pose pickup1Pose = new Pose(10.5, 65.5, Math.toRadians(180));
        private final Pose pickup1CP1 = new Pose(50, 65, Math.toRadians(180));
        private final Pose pickup1CP2 = new Pose(40, 65.5, Math.toRadians(180));
    //line 4 Score 1 -------------------------------------------------------------------------------
        private final Pose score1Pose = new Pose(21.68, 96.6776, Math.toRadians(135));
    //line 6 Pickup  2 -----------------------------------------------------------------------------
        private final Pose pickup2Pose = new Pose(5, 42.5, Math.toRadians(180));
        private final Pose pickup2CP1 = new Pose(44, 42.5, Math.toRadians(180));
        private final Pose pickup2CP2 = new Pose(33, 42.5, Math.toRadians(180));
    //line 7 Push Bar ------------------------------------------------------------------------------
        private final Pose pushBarPose = new Pose(16.5, 80, Math.toRadians(180));
        private final Pose pushBarCP1 = new Pose(25, 80, Math.toRadians(180));
    //line 8 Score  2 ------------------------------------------------------------------------------
        private final Pose score2Pose = new Pose(21.68, 96.6776, Math.toRadians(135));
        private final Pose score2CP1 = new Pose(24,43, Math.toRadians(180));
        private final Pose score2CP2 = new Pose(22.4, 70, Math.toRadians(150));
    //line 9 Pickup  3------------------------------------------------------------------------------
        //private final Pose pickup3Pose = new Pose(17, 115, Math.toRadians(180));
        //private final Pose pickup3CP1 = new Pose(50, 90, Math.toRadians(180));
       // private final Pose pickup3CP2 = new Pose(45, 110, Math.toRadians(180));
    //line 10 Score 3-------------------------------------------------------------------------------
       // private final Pose score3Pose = new Pose(32, 30, Math.toRadians(225));
      //  private final Pose score3CP1 = new Pose(25, 90, Math.toRadians(180));
      //  private final Pose score3CP2 = new Pose(30, 65, Math.toRadians(190));
    //line 10 Park----------------------------------------------------------------------------------
        private final Pose park = new Pose(22.5, 54, Math.toRadians(180));
    //  private PathChain ;-------------------------------------------------------------------------
        private Path scorePreload,  Pickup1,Score1,Pickup2,PushBar,Score2,Pickup3,Score3,Park;
//--------------------------------------------------------------------------------------------------
        public void buildPaths() {
//line 1 --------------------------------------------------------------------------------------------
            scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(scorePose)));
            scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//Line 2 ---------------------------------------------------------------------------------------------------------------
            Pickup1 = new Path(new BezierCurve(new Point(scorePose), new Point(pickup1CP1), new Point(pickup1CP2), new Point(pickup1Pose)));
            Pickup1.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading());
//line 3 ---------------------------------------------------------------------------------------------------------------
            Score1 = new Path(new BezierCurve(new Point(pickup1Pose), new Point(score1Pose)));
            Score1.setLinearHeadingInterpolation(pickup1Pose.getHeading(), score1Pose.getHeading());
//line 4 ----------------------------------------------------------------------------------------------------------------------------------
            Pickup2 = new Path(new BezierCurve(new Point(score1Pose), new Point(pickup2CP1), new Point(pickup2CP2), new Point(pickup2Pose)));
            Pickup2.setLinearHeadingInterpolation(score1Pose.getHeading(), pickup2Pose.getHeading());
//line 5 ----------------------------------------------------------------------------------------------------------------------------------
            //PushBar = new Path(new BezierCurve(new Point(pickup2Pose), new Point(pushBarCP1), new Point(pushBarPose)));
            //PushBar.setLinearHeadingInterpolation(pickup2Pose.getHeading(), pushBarPose.getHeading());
//line 6 ----------------------------------------------------------------------------------------------------------------------------------
            Score2 = new Path(new BezierCurve(new Point(pickup2Pose), new Point(score2CP1), new Point(score2CP2), new Point(score2Pose)));
            Score2.setLinearHeadingInterpolation(pickup2Pose.getHeading(), score2Pose.getHeading());
//line 7 ----------------------------------------------------------------------------------------------------------------------------------
            //Pickup3 = new Path(new BezierCurve(new Point(score2Pose), new Point(pickup3CP1), new Point(pickup3CP2), new Point(pickup3Pose)));
           //Pickup3.setLinearHeadingInterpolation(score2Pose.getHeading(), pickup3Pose.getHeading());
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
                    follower.followPath(scorePreload, true);
                    setPathState(1);
                    shotTimer.reset();
                    break; // --------------------------------------Shoots balls1--------------------------------------------
                case 1:
                    ((DcMotorEx) flywheel).setVelocity(bankVelocity);
                    if ((!follower.isBusy()) && ((DcMotorEx) flywheel).getVelocity() >= bankVelocity - 5) {//starts shooter
                           led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                            intake.setPower(1);
                    }
                    if(shotTimer.milliseconds() > 2700) {
                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        BootKick.setPosition(0.5);
                        }
                    if(shotTimer.milliseconds() > 3700) {
                       led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        BootKick.setPosition(0);
                        setPathState(2);
                    }
                    break; // --------------------------------------Picks up 1st line ---------------------------------------
                case 2:
                    follower.followPath(Pickup1, true);
                    ((DcMotorEx) flywheel).setVelocity(0);//turns shooter off
                    setPathState(3);
                    break; // -------------------------------------------------------------------------------------------
                case 3:
                    ballrelease.setPosition(0.45);
                    follower.setMaxPower(0.6);
                    if(slowDownTimer.milliseconds() > 1000 ){
                        follower.setMaxPower(0.4);
                        intake.setPower(1);
                    }
                    intake.setPower(1); //turns intake on
                    if (!follower.isBusy()) {
                        setPathState(4);
                        follower.setMaxPower(0.4);
                    }
                    break; // -------------------------------------Moves to Score--------------------------------
                case 4:
                    follower.setMaxPower(0.95);
                    follower.followPath(Score1, true);
                    ballrelease.setPosition(0.3);
                    intake.setPower(0);
                    flywheel.setPower(0);
                    setPathState(5);//starts shooter
                    shotTimer.reset();


                    break; // --------------------------------Shots Balls2-------------------------------------------
                case 5:
                    if (!follower.isBusy()) {
                        //starts shooter
                        ((DcMotorEx) flywheel).setVelocity(bankVelocity);
                        if (((DcMotorEx) flywheel).getVelocity() >= bankVelocity - 5) {
                            ballrelease.setPosition(0.3);
                            intake.setPower(1);
                        }
                        if(slowDownTimer.milliseconds() > 2000 ){
                            ((DcMotorEx) flywheel).setVelocity(bankVelocity);
                        }
                    }
                    if(shotTimer.milliseconds() > 4000)  {
                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        BootKick.setPosition(0.5);
                    }
                    if(shotTimer.milliseconds() > 4800) {
                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        BootKick.setPosition(0.0);
                        setPathState(6);
                    }
                    slowDownTimer.reset();
                    break; // -----------------------------------Picks up 2nd Line----------------------------------------
                case 6:
                    follower.followPath(Pickup2, true);
                    intake.setPower(0);//turns shooter off
                    flywheel.setPower(0);
                    setPathState(7);
                    slowDownTimer.reset();
                    break; // -------------------------------------------------------------------------------------------
                case 7:
                    if(slowDownTimer.milliseconds() > 1600) {
                        ballrelease.setPosition(0.45);
                        follower.setMaxPower(.4);
                        intake.setPower(1);
                    }
                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                    if (!follower.isBusy()) {
                        setPathState(8);
                        intake.setPower(0);
                    }
                    break; // ----------------------------------Hits Push Bar-----------------------------------
                case 8:
                    follower.setMaxPower(0.95);
                    /* follower.followPath(PushBar, true);*/
                    setPathState(9);
                    break; // -------------------------------------------------------------------------------------------
                case 9:

                    if (!follower.isBusy()) {
                        intake.setPower(0);
                        setPathState(10);
                    }
                    break; // --------------------------------Moves to Score--------------------------------------------
                case 10:
                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
                    intake.setPower(0);
                    follower.setMaxPower(0.95);
                    follower.followPath(Score2, true);
                    setPathState(11);
                    shotTimer.reset();
                    break; // --------------------------------Shots Balls3--------------------------------------------
                case 11:
                    ballrelease.setPosition(0.3);
                    if (!follower.isBusy()) {
                        //starts shooter
                        ((DcMotorEx) flywheel).setVelocity(bankVelocity);
                        if (((DcMotorEx) flywheel).getVelocity() >= bankVelocity - 5) {
                            intake.setPower(1);
                        }
                    }
                    if(shotTimer.milliseconds() > 5200) {
                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        BootKick.setPosition(0.5);
                    }
                    if(shotTimer.milliseconds() > 5700) {
                        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        BootKick.setPosition(0.0);
                        setPathState(12);
                    }
                    slowDownTimer.reset();
                    break; // -------------------------------Picks up 3rd Line--------------------------------------------
                case 12:
                    follower.followPath(Park, true);
                    intake.setPower(0);
                     //turns shooter off
                    flywheel.setPower(0);
                    setPathState(17);
                    slowDownTimer.reset();
                    break; // -------------------------------------------------------------------------------------------
                case 13:
                    if(slowDownTimer.milliseconds() > 2000) {
                        follower.setMaxPower(.5);
                        ballrelease.setPosition(0.45);
                        intake.setPower(1);
                    }
                    if (!follower.isBusy()) {
                        intake.setPower(0);
                        setPathState(14);
                    }
                    break; // -------------------------------------------------------------------------------------------
                case 14:
                    follower.setMaxPower(0.95);
                    follower.followPath(Score3, true);
                    setPathState(15);
                    shotTimer.reset();
                    break; // -------------------------------------------------------------------------------------------
                case 15:
                    if (!follower.isBusy()) {
                        ballrelease.setPosition(0.3);
                        ((DcMotorEx) flywheel).setVelocity(bankVelocity);
                        if (((DcMotorEx) flywheel).getVelocity() >= bankVelocity - 5) {
                            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                            intake.setPower(1);

                        }
                    }
                    if(shotTimer.milliseconds() > 2800) {
                       led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        BootKick.setPosition(0.5);
                    }
                    if(shotTimer.milliseconds() > 4400) {
                       led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        BootKick.setPosition(0.0);
                        setPathState(16);
                    }
                    break; // -------------------------------------------------------------------------------------------
                case 16:
                    follower.followPath(Park, true);
                    setPathState(17);
                    break; // -------------------------------------------------------------------------------------------
                case 17:
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

            telemetry.update();
            intake = hardwareMap.get(DcMotor.class, "intake");
            turret = hardwareMap.get(DcMotor.class, "turret");
            flywheel = hardwareMap.get(DcMotor.class, "flywheel");
            ballrelease = hardwareMap.get(Servo.class,"ballrelease");
            BootKick = hardwareMap.get(Servo.class,"BootKick");
            led = hardwareMap.get(RevBlinkinLedDriver.class,"led");
           // shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
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

