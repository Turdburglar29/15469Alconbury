package pedroPathing.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class BlueTurretTracker {

    public final double TICKS_PER_REV = 537.6898;   // GoBILDA 312 RPM
    public final double GEAR_RATIO = 62.0 / 14.0;
    public final double TICKS_PER_RAD = (TICKS_PER_REV * GEAR_RATIO) / (2.0 * Math.PI);

    private final DcMotor turret;

    // Forbidden arc 170-190 degrees in radians
    private static final double MIN_ARC = Math.toRadians(170);
    private static final double MAX_ARC = Math.toRadians(190);

    public BlueTurretTracker(DcMotor turretMotor) {
        this.turret = turretMotor;
    }

    /**
     * Calculate safe turret angle relative to robot to target goal, avoiding forbidden arc
     * @param robotX current robot x
     * @param robotY current robot y
     * @param robotHeading current robot heading (radians)
     * @param goalX goal x coordinate
     * @param goalY goal y coordinate
     * @return turret target angle in radians
     */
    public double calculateTargetAngle(double robotX, double robotY, double robotHeading, double goalX, double goalY) {
        double rawTarget = Math.atan2(goalY - robotY, goalX - robotX) - robotHeading;
        rawTarget = normalizeAngle(rawTarget);

        // Forbidden arc avoidance: if target is in 170-190 deg range, rotate clockwise past 190
        if (rawTarget >= MIN_ARC && rawTarget <= MAX_ARC) {
            rawTarget = MAX_ARC + 0.01; // small offset to move past
        }

        return rawTarget;
    }

    /** Normalize angle to [-pi, pi] */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}