package pedroPathing.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "DecodeAuto", group = "Examples")
    public class DecodeGGPAuto extends OpMode {

        private ElapsedTime runtime = new ElapsedTime();
        private ElapsedTime clipTimer = new ElapsedTime();
        static final double COUNTS_PER_MOTOR_REV = 537.6898;   // goBilda 5202 Motor Encoder
        static final double DRIVE_GEAR_REDUCTION = 19.2032;     // goBilda 5202 Gear ratio reduction
        static final double WHEEL_DIAMETER_INCHES = 3.77953;     // goBilda 5202 Wheel diameter
        static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
        static final double DRIVE_SPEED = 0.4;
        static final double TURN_SPEED = 0.5;
        private Follower follower;
        private Timer pathTimer, actionTimer, opmodeTimer;
        double startTime;
        private int pathState;
        //Start point
        private final Pose startPose = new Pose(5, 81, Math.toRadians(180));
        //line 1 ScorePreload 1
        private final Pose scorePose = new Pose(49.4, 81, Math.toRadians(180));
        //line 2 PushSample 1
        private final Pose push1Pose = new Pose(13, 12, Math.toRadians(0));
        private final Pose push1CP1 = new Pose(13, 81, Math.toRadians(0));
        private final Pose push1CP2 = new Pose(8, 12, Math.toRadians(0));
        private final Pose push1CP3 = new Pose(70, 39, Math.toRadians(0));
        private final Pose push1CP4 = new Pose(180, 12, Math.toRadians(0));
        //Line 3 Pickup specimen 1
        private final Pose pickup1Pose = new Pose(0.2, 32, Math.toRadians(0));
        private final Pose pickup1CP1 = new Pose(25, 32, Math.toRadians(0));
        private final Pose pickup1CP2 = new Pose(13, 32, Math.toRadians(0));
        //line 4 Score speciman 1
        private final Pose score1Pose = new Pose(50, 84, Math.toRadians(180));
        private final Pose score1CP1 = new Pose(25, 37, Math.toRadians(0));
        private final Pose score1CP2 = new Pose(30, 52, Math.toRadians(90));
        private final Pose score1CP3 = new Pose(40, 67, Math.toRadians(160));
        //line 5 Push Sample 2
        private final Pose push2Pose = new Pose(13, 0, Math.toRadians(0));
        private final Pose push2CP1 = new Pose(8, 10, Math.toRadians(0));
        private final Pose push2CP2 = new Pose(70, 20, Math.toRadians(0));
        private final Pose push2CP3 = new Pose(180, 0, Math.toRadians(0));
        //line 6 Pickup specimen 2
        private final Pose pickup2Pose = new Pose(3.4, 33, Math.toRadians(0));
        private final Pose pickup2CP1 = new Pose(10, 33, Math.toRadians(0));
        private final Pose pickup2CP2 = new Pose(5, 33, Math.toRadians(0));
        //line 7 Score specimen 2
        private final Pose score2Pose = new Pose(49, 90, Math.toRadians(180));
        private final Pose score2CP1 = new Pose(25, 37, Math.toRadians(90));
        private final Pose score2CP2 = new Pose(40, 67, Math.toRadians(160));
        //line 8 Pickup specimen 3
        private final Pose pickup3Pose = new Pose(3.4, 33, Math.toRadians(0));
        private final Pose pickup3CP1 = new Pose(25, 33, Math.toRadians(0));
        private final Pose pickup3CP2 = new Pose(15, 33, Math.toRadians(0));
        //line 9 Score specimen 3
        private final Pose score3Pose = new Pose(49, 85, Math.toRadians(180));
        private final Pose score3CP1 = new Pose(25, 37, Math.toRadians(0));
        private final Pose score3CP2 = new Pose(30, 55, Math.toRadians(90));
        private final Pose score3CP3 = new Pose(40, 67, Math.toRadians(160));
        //line 10 Park
        private final Pose park = new Pose(3.4, 33, Math.toRadians(0));
        private Path scorePreload, Push1, Pickup1, Score1, Push2, Pickup2, Score2, Pickup3,Park;
        //  private PathChain ;

        public void buildPaths() {
            //line 1
            scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
            scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//line 2
            Push1 = new Path(new BezierCurve(new Point(scorePose), new Point(push1CP1), new Point(push1CP2), new Point(push1CP3), new Point(push1CP4), new Point(push1Pose)));
            Push1.setLinearHeadingInterpolation(scorePose.getHeading(), push1Pose.getHeading());
//Line 3
            Pickup1 = new Path(new BezierCurve(new Point(push1Pose), new Point(pickup1CP1), new Point(pickup1CP2), new Point(pickup1Pose)));
            Pickup1.setLinearHeadingInterpolation(push1Pose.getHeading(), pickup1Pose.getHeading());
//line 4
            Score1 = new Path(new BezierCurve(new Point(pickup1Pose), new Point(score1CP1), new Point(score1CP2), new Point(score1CP3), new Point(score1Pose)));
            Score1.setLinearHeadingInterpolation(pickup1Pose.getHeading(), score1Pose.getHeading());
//line 5
            Push2 = new Path(new BezierCurve(new Point(score1Pose), new Point(push2CP1), new Point(push2CP2), new Point(push2CP3), new Point(push2Pose)));
            Push2.setLinearHeadingInterpolation(score1Pose.getHeading(), push2Pose.getHeading());
//line 6
            Pickup2 = new Path(new BezierCurve(new Point(push2Pose), new Point(pickup2CP1), new Point(pickup2CP2), new Point(pickup2Pose)));
            Pickup2.setLinearHeadingInterpolation(push2Pose.getHeading(), pickup2Pose.getHeading());
//line 7
            Score2 = new Path(new BezierCurve(new Point(pickup2Pose), new Point(score2CP1), new Point(score2CP2), new Point(score2Pose)));
            Score2.setLinearHeadingInterpolation(pickup2Pose.getHeading(), score2Pose.getHeading());
//line 8
            Pickup3 = new Path(new BezierCurve(new Point(score2Pose), new Point(pickup3CP1), new Point(pickup3CP2), new Point(pickup3Pose)));
            Pickup3.setLinearHeadingInterpolation(score2Pose.getHeading(), pickup3Pose.getHeading());

            Park = new Path(new BezierLine(new Point(score3Pose), new Point(park)));

        }
        public void autonomousPathUpdate() {
            switch (pathState) {
                case 0:

                    setPathState(1);
                    follower.followPath(scorePreload);
                break;
                case 1:
                        setPathState(2);

                break;
                case 2:
                        setPathState(3);
                        follower.followPath(Push1, true);

                break;
                case 3:
                        follower.followPath(Pickup2, true);
                        setPathState(4);

                break;
                case 4:
                        setPathState(5);

                break;
                case 5:
                        follower.followPath(Score1, true);
                        setPathState(6);

                break;
                case 6:
                        setPathState(7);

                break;
                case 7:
                        setPathState(8);

                break;
                case 8:
                        follower.followPath(Pickup3, true);
                        setPathState(9);

                break;
                case 9:
                        setPathState(5);

                break;
                case 10:
                        follower.followPath(Score2, true);
                        setPathState(11);

                break;
                case 11:
                        setPathState(12);

                break;
                case 12:
                        setPathState(13);
                        follower.followPath(Park, true);

                        setPathState(-1);
                break;
                case -1:
                        stop();


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

