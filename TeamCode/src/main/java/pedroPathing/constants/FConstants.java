package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import  com.qualcomm.robotcore.hardware.DcMotor;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightRear";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;


        FollowerConstants.mass = 6
        ; //robot mass in kg - Its a Fatty Fat...FAT FAT

        FollowerConstants.xMovement = 73.5669; // 61.043622825611145 at 255 2/17
        FollowerConstants.yMovement = 46.10735; // 42.621403162065704 at 255 2/17 43.6631

        FollowerConstants.forwardZeroPowerAcceleration = -19.1767; // -27.941003214676822 at 255 2/17
        FollowerConstants.lateralZeroPowerAcceleration = -41.5858; // -82.32803462012504 at 255 2/17

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.15,0.0001,0.001,0); // 0.2,0.0001,0.01,0 at 255 2/17
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(2,0,1,0); // Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0.04,0); // 2,0,0.1,0 at 255 2/17
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(0,0,0,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.015,0,0,0.6,0); // 0.015,0,0,0.6,0 at 255 2/17
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0,0,0,0,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4; // 4 at 255 2/17
        FollowerConstants.centripetalScaling = 0.0001; // 0.0001 at 255 2/17

        FollowerConstants.pathEndTimeoutConstraint = 500; // 500 at 255 2/17
        FollowerConstants.pathEndTValueConstraint = 0.995; // 0.995 at 255 2/17
        FollowerConstants.pathEndVelocityConstraint = 0.1; // 0.1 at 255 2/17
        FollowerConstants.pathEndTranslationalConstraint = 0.1; // 0.1 at 255 2/17
        FollowerConstants.pathEndHeadingConstraint = 0.007; // 0.007 at 255 2/17
    }
}
