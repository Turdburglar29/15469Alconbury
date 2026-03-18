package pedroPathing.subsystems;

import com.pedropathing.localization.Pose;

/**
 * Fuses odometry pose with Limelight pose.
 * - Hard snap if drift is large
 * - Smooth blend otherwise
 */
public class PoseCorrector {

    // Tunables
    public double snapThresholdInches = 6.0;   // position snap threshold
    public double snapThresholdDeg = 15.0;     // heading snap threshold
    public double smoothing = 0.15;            // 0..1, higher = faster correction

    public Pose fuse(Pose odo, double camX_in, double camY_in, double camHeadingRad) {
        double dx = camX_in - odo.getX();
        double dy = camY_in - odo.getY();
        double dHeadRad = camHeadingRad - odo.getHeading();

        double dist = Math.hypot(dx, dy);
        double dHeadDeg = Math.toDegrees(dHeadRad);

        // Hard snap if drift is large
        if (dist > snapThresholdInches || Math.abs(dHeadDeg) > snapThresholdDeg) {
            return new Pose(camX_in, camY_in, camHeadingRad);
        }

        // Smooth correction
        double fusedX = odo.getX() + dx * smoothing;
        double fusedY = odo.getY() + dy * smoothing;
        double fusedH = odo.getHeading() + dHeadRad * smoothing;

        return new Pose(fusedX, fusedY, fusedH);
    }
}
