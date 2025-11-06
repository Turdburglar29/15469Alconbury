package pedroPathing.tuners_tests.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Config
@Autonomous (name = "Curved Back And Forth", group = "PIDF Testing")
public class CurvedBackAndForth extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 0;

    private boolean forward = true;

    private Follower follower;

    private Path forwards;
    private Path backwards;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);

        forwards = new Path(new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(Math.abs(DISTANCE),0, Point.CARTESIAN), new Point(Math.abs(DISTANCE),DISTANCE, Point.CARTESIAN)));
        backwards = new Path(new BezierCurve(new Point(Math.abs(DISTANCE),DISTANCE, Point.CARTESIAN), new Point(Math.abs(DISTANCE),0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));

        backwards.setReversed(true);

        follower.followPath(forwards);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a curve going " + DISTANCE + " inches"
                            + " to the left and the same number of inches forward. The robot will go"
                            + "forward and backward continuously along the path. Make sure you have"
                            + "enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }

        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
}
