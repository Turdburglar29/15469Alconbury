package pedroPathing.constants;

import com.pedropathing.localization.localizers.DriveEncoderLocalizer;
import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.pedropathing.localization.constants.DriveEncoderConstants;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
        DriveEncoderLocalizer.TURN_TICKS_TO_RADIANS = 6.4846;
        DriveEncoderLocalizer.FORWARD_TICKS_TO_INCHES = 72.8025;
        DriveEncoderLocalizer.STRAFE_TICKS_TO_INCHES = 71.4431;
        PinpointConstants.forwardY = -1.88976;
        PinpointConstants.strafeX = -5.055118;
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        PinpointConstants.customEncoderResolution = 19.89436789;//this is only used if using custom deadwheels
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
    }
}




