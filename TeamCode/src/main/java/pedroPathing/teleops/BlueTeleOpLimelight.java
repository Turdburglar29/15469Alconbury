package pedroPathing.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstantsTeleop;
import pedroPathing.subsystems.TurretController;

// === Limelight relocalization ===
import pedroPathing.subsystems.Limelight3A;
import pedroPathing.subsystems.PoseCorrector;
@Disabled
@TeleOp(name = "BlueTeleOplimelight", group = "BlueTeleOp")
public class BlueTeleOpLimelight extends OpMode {

    private Follower follower;

    // (Kept your hardware declarations unchanged, even though shooter/intake no longer used)
    private DcMotor flywheel;
    private DcMotor intake;
    private Servo ballrelease;
    private Servo BootKick;
    private RevBlinkinLedDriver led;

    private final ElapsedTime shotTimer = new ElapsedTime(); // no longer used for shooting; safe to keep
    private boolean lastCircle = false; // leftover; safe to keep if you later rebind buttons

    private static final int IDLVelocity = 500; // unused now
    private static final int bankVelocity = 1000; // unused now
    private static final int medVelocity = 1400; // unused now
    private static final int farVelocity = 2200; // unused now
    private static final int maxVelocity = 2000; // unused now

    private TurretController turret;

    // === Limelight pose-correction ===
    private static final String LIMELIGHT_IP = "192.168.1.11"; // TODO: set to your Limelight 3A IP
    private Limelight3A limelight;
    private PoseCorrector poseCorrector;

    // === Triangle-triggered multi-sample burst state ===
    private boolean lastTriangle = false;
    private boolean llBurstActive = false;
    private int llBurstSamplesTaken = 0;
    private int llBurstGood = 0;
    private long llBurstStartMs = 0L;

    // Accumulators for averaging (position simple mean; heading circular mean)
    private double sumX_in = 0.0;
    private double sumY_in = 0.0;
    private double sumSinH = 0.0;
    private double sumCosH = 0.0;

    // Tunables for the burst
    private static final int LL_BURST_MAX_SAMPLES = 5;     // take up to 5 frames
    private static final int LL_BURST_TIMEOUT_MS = 150;    // or stop after 150 ms
    private static final int LL_BURST_MIN_GOOD = 1;        // need at least 1 good frame to apply

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstantsTeleop.class);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(30, 75, Math.toRadians(180)));

        /* === TURRET INIT === */
        turret = new TurretController(hardwareMap, "turret", follower);
        turret.setHeadingCcwPositive(false);
        turret.setTickSoftLimitsEnabled(true);
        turret.setTickLimits(-385, 320); // your requested soft stops
        turret.setSoftMarginTicks(3);
        turret.setSlowZoneTicks(50);
        turret.setMountOffsetRad(Math.toRadians(-173));

        /* === (Shooter/Intake HW kept but unused) === */
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        ballrelease = hardwareMap.get(Servo.class, "ballrelease");
        BootKick = hardwareMap.get(Servo.class, "BootKick");
        led = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        // === Limelight init ===
        //limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight = new Limelight3A(LIMELIGHT_IP);
        poseCorrector = new PoseCorrector();

        telemetry.addLine("Init OK");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        /* ---------------- DRIVE ---------------- */
        follower.setTeleOpMovementVectors(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
        follower.update(); // keep pose fresh

        // One-button calibration: manually aim at the real goal, then press OPTIONS once
        if (gamepad1.options) {
            turret.calibrateMountOffsetToCurrentAim();
        }

        /* ---------------- (SHOOTER & INTAKE REMOVED) ---------------- */
        // Replaced by triangle-triggered multi-sample relocalization burst below.

        /* ---------------- TURRET UPDATE (ALWAYS) ---------------- */
        if (turret != null) {
            turret.update();
        }

        /* ------- LIMELIGHT POSE CORRECTION (TRIANGLE, MULTI-SAMPLE BURST) ------- */

        // Start burst on triangle rising edge
        boolean trianglePressed = gamepad1.triangle;
        if (trianglePressed && !lastTriangle && !llBurstActive) {
            llBurstActive = true;
            llBurstSamplesTaken = 0;
            llBurstGood = 0;
            llBurstStartMs = System.currentTimeMillis();
            sumX_in = sumY_in = sumSinH = sumCosH = 0.0;
        }
        lastTriangle = trianglePressed;

        // While burst active, collect samples non-blockingly across loop iterations
        if (llBurstActive) {
            long elapsed = System.currentTimeMillis() - llBurstStartMs;

            // Attempt a new sample this iteration (one per loop)
            if (llBurstSamplesTaken < LL_BURST_MAX_SAMPLES && elapsed <= LL_BURST_TIMEOUT_MS) {
                limelight.update();
                llBurstSamplesTaken++;
                if (limelight.hasValidTarget()) {
                    double x = limelight.getX_in();
                    double y = limelight.getY_in();
                    double h = limelight.getHeadingRad();
                    sumX_in += x;
                    sumY_in += y;
                    sumSinH += Math.sin(h);
                    sumCosH += Math.cos(h);
                    llBurstGood++;
                }
            }

            // Decide to end the burst (enough samples or timed out)
            if (llBurstSamplesTaken >= LL_BURST_MAX_SAMPLES || elapsed > LL_BURST_TIMEOUT_MS) {
                if (llBurstGood >= LL_BURST_MIN_GOOD) {
                    // Compute averages
                    double avgX = sumX_in / llBurstGood;
                    double avgY = sumY_in / llBurstGood;
                    double avgH = Math.atan2(sumSinH / llBurstGood, sumCosH / llBurstGood);

                    Pose odoPose = follower.getPose();
                    Pose corrected = poseCorrector.fuse(
                            odoPose,
                            avgX,
                            avgY,
                            avgH
                    );
                    follower.setPose(corrected);

                    telemetry.addLine("LL Pose Applied (Triangle burst)");
                    telemetry.addData("Samples", "%d/%d", llBurstGood, llBurstSamplesTaken);
                    telemetry.addData("Avg X (in)", avgX);
                    telemetry.addData("Avg Y (in)", avgY);
                    telemetry.addData("Avg H (deg)", Math.toDegrees(avgH));
                } else {
                    telemetry.addLine("Triangle burst: No LL targets");
                }
                // end burst
                llBurstActive = false;
            }
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

        telemetry.addData("MountOffset(deg)", Math.toDegrees(turret.getMountOffsetRad()));
        telemetry.addData("DesiredTicks", desiredTicks);
        telemetry.addData("ActualTicks", actualTicks);
        telemetry.addData("ErrorTicks", (desiredTicks - actualTicks));
        telemetry.addData("Pose", "(%.1f, %.1f, %.1f°)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("Goal", "(%.1f, %.1f)", TurretController.GOAL_X, TurretController.GOAL_Y);

        // Burst status telemetry
        telemetry.addData("LL Burst", llBurstActive ? "ACTIVE" : "idle");
        if (llBurstActive) {
            telemetry.addData("LL Burst Samples", "%d/%d", llBurstGood, llBurstSamplesTaken);
            telemetry.addData("LL Burst Elapsed (ms)", System.currentTimeMillis() - llBurstStartMs);
        }

        telemetry.update();
    }
}