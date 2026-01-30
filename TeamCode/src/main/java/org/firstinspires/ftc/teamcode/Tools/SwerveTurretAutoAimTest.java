package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "SwerveTurretAutoAimTest", group = "Tools")
public class SwerveTurretAutoAimTest extends LinearOpMode {

    /* ===================== TURRET SERVOS ===================== */
    private Servo turretRotation1, turretRotation2;

    private static final double MIN_TURRET = 0.0;
    private static final double MAX_TURRET = 1.0;

    // Camera mounted vertically (portrait): use tag.center.y as your aim axis
    private static final boolean CAMERA_IS_PORTRAIT = true;

    // Flip this if the turret moves the wrong way with auto aim
    private static final boolean INVERT_AIM_AXIS = false;

    // Final sign flip for turret hardware direction (usually +1)
    private static final double AIM_SIGN = +1.0;

    // Optional gear compensation (start false)
    private static final double GEAR_TO_TURRET_COMP = 1.0 / 0.7885; // ~1.268
    private static final boolean USE_GEAR_COMP = false;

    // === "BUTTERY SMOOTH" TUNING ===
    // Deadband: how close is "good enough"
    private static final double PIXEL_DEADBAND = 18.0;

    // Only flip direction if error got meaningfully worse
    private static final double WORSE_HYST_PX = 4.0;

    // Step size: we SCALE this with error for smooth approach
    private static final double STEP_MIN = 0.0008;  // minimum movement to avoid stalling
    private static final double STEP_MAX = 0.0045;  // MUST be <= 0.005 per your request

    // Error scale: bigger value = slower movement overall (smoother)
    private static final double ERR_SCALE_PX = 140.0;

    // Simple smoothing filter on tag center to reduce jitter
    // (EMA: exponential moving average)
    private static final double EMA_ALPHA = 0.25; // 0.15 smoother, 0.35 more responsive

    // Main turret position variable (your "position 1")
    private double turretPos1 = 0.5;

    // Direction lock: once we find improvement, keep moving this way
    private int aimDir = +1;

    // Track error trend
    private double prevAbsErrorPx = Double.NaN;

    // Filter state for measured center pixel
    private double filtMeasured = Double.NaN;

    /* ===================== CAMERA / APRILTAG ===================== */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // DECODE goal IDs
    private static final int TAG_BLUE_GOAL = 20;
    private static final int TAG_RED_GOAL  = 24;
    private int targetId = TAG_RED_GOAL;

    // Auto aim enable toggle
    private boolean autoAimEnabled = true;
    private boolean togglePrev = false;

    // Midpoints assume 640x480 stream.
    // If your stream differs, update these.
    private static final double FRAME_MID_Y = 240.0;
    private static final double FRAME_MID_X = 320.0;

    /* ===================== DEBUG / TIMING ===================== */
    private long lastSeenMs = 0;
    private long prevLoopMs = 0;
    private long loopDtMs = 0;

    @Override
    public void runOpMode() {

        initHardware();
        initVision();

        telemetry.addLine("TurretAutoAim_SimplePixel_DEBUG ready (BUTTERY SMOOTH build)");
        telemetry.addLine("Uses webcam name: turretCam");
        telemetry.addLine("GP2: LB toggle autoaim | dpad_left=BLUE | dpad_right=RED | RSX manual when autoaim OFF");
        telemetry.addLine("Auto Aim: EMA-smoothed pixel + scaled micro-step + direction-lock");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            long now = System.currentTimeMillis();
            if (prevLoopMs == 0) prevLoopMs = now;
            loopDtMs = now - prevLoopMs;
            prevLoopMs = now;

            /* ===================== CONTROLS ===================== */
            if (gamepad2.dpad_left)  targetId = TAG_BLUE_GOAL;
            if (gamepad2.dpad_right) targetId = TAG_RED_GOAL;

            boolean toggle = gamepad2.left_bumper;
            if (toggle && !togglePrev) autoAimEnabled = !autoAimEnabled;
            togglePrev = toggle;

            AprilTagDetection det = getDetectionForId(targetId);

            /* ===================== DEBUG VARIABLES ===================== */
            boolean tagVisible = (det != null);

            double cx = Double.NaN;
            double cy = Double.NaN;

            double measured = Double.NaN;
            double filtered = Double.NaN;

            double errorPx = Double.NaN;
            double absErr = Double.NaN;

            boolean inDeadband = false;
            String moveDirText = "NONE";

            double appliedDelta = 0.0;
            double turretBefore = turretPos1;

            double stepUsed = 0.0;
            boolean flippedThisLoop = false;

            /* ===================== AIM LOGIC ===================== */
            if (!autoAimEnabled) {
                // Manual turret control when autoaim OFF
                double manual = -gamepad2.right_stick_x;
                if (Math.abs(manual) < 0.05) manual = 0;

                appliedDelta = manual * 0.01;
                turretPos1 = clamp(turretPos1 + appliedDelta, MIN_TURRET, MAX_TURRET);
                moveDirText = (manual == 0) ? "MANUAL HOLD" : "MANUAL";

                // Reset tracking memory so autoaim doesn't inherit stale direction
                prevAbsErrorPx = Double.NaN;
                filtMeasured = Double.NaN;

            } else {
                // Auto aim ON
                if (tagVisible) {
                    cx = det.center.x;
                    cy = det.center.y;

                    measured = CAMERA_IS_PORTRAIT ? cy : cx;
                    double mid = CAMERA_IS_PORtrait() ? FRAME_MID_Y : FRAME_MID_X;

                    // EMA filter for smoothness
                    if (Double.isNaN(filtMeasured)) filtMeasured = measured;
                    filtMeasured = (EMA_ALPHA * measured) + ((1.0 - EMA_ALPHA) * filtMeasured);
                    filtered = filtMeasured;

                    errorPx = (filtered - mid);
                    if (INVERT_AIM_AXIS) errorPx = -errorPx;

                    absErr = Math.abs(errorPx);
                    inDeadband = absErr <= PIXEL_DEADBAND;
                    lastSeenMs = now;

                    if (inDeadband) {
                        moveDirText = "DEADBAND (HOLD)";
                        appliedDelta = 0.0;
                        prevAbsErrorPx = absErr;
                    } else {
                        // Seed previous error on first acquire
                        if (Double.isNaN(prevAbsErrorPx)) prevAbsErrorPx = absErr;

                        // If it got worse compared to last frame, flip direction
                        if (absErr > prevAbsErrorPx + WORSE_HYST_PX) {
                            aimDir *= -1;
                            flippedThisLoop = true;
                        }

                        // Scaled step: faster when far, slower near center (smooth)
                        double scale = absErr / ERR_SCALE_PX;   // e.g. 140px => ~1.0
                        double step = STEP_MAX * scale;
                        step = clamp(step, STEP_MIN, STEP_MAX);

                        if (USE_GEAR_COMP) step *= GEAR_TO_TURRET_COMP;
                        // Still respect <= 0.005 hard requirement
                        step = Math.min(step, 0.0050);

                        stepUsed = step;

                        appliedDelta = AIM_SIGN * aimDir * step;
                        turretPos1 = clamp(turretPos1 + appliedDelta, MIN_TURRET, MAX_TURRET);

                        prevAbsErrorPx = absErr;

                        if (flippedThisLoop) moveDirText = "WORSE->FLIP then STEP";
                        else moveDirText = (aimDir > 0) ? "DIR + STEP" : "DIR - STEP";
                    }
                } else {
                    moveDirText = "NO TAG (HOLD)";
                    appliedDelta = 0.0;

                    prevAbsErrorPx = Double.NaN;
                    filtMeasured = Double.NaN;
                }
            }

            /* ===================== APPLY SERVOS ===================== */
            turretRotation1.setPosition(turretPos1);
            turretRotation2.setPosition(1.0 - turretPos1);

            /* ===================== TELEMETRY ===================== */
            telemetry.addData("AutoAim", autoAimEnabled ? "ON" : "OFF");
            telemetry.addData("Target", targetId == TAG_RED_GOAL ? "RED (24)" : "BLUE (20)");

            telemetry.addData("CAMERA_IS_PORTRAIT", CAMERA_IS_PORTRAIT);
            telemetry.addData("INVERT_AIM_AXIS", INVERT_AIM_AXIS);
            telemetry.addData("AIM_SIGN", "%.1f", AIM_SIGN);

            telemetry.addData("EMA_ALPHA", "%.2f", EMA_ALPHA);
            telemetry.addData("DEADBAND(px)", "%.1f", PIXEL_DEADBAND);
            telemetry.addData("WORSE_HYST(px)", "%.1f", WORSE_HYST_PX);

            telemetry.addData("STEP_MIN/MAX", "%.4f / %.4f", STEP_MIN, STEP_MAX);
            telemetry.addData("ERR_SCALE_PX", "%.1f", ERR_SCALE_PX);
            telemetry.addData("stepUsed", "%.5f", stepUsed);
            telemetry.addData("USE_GEAR_COMP", USE_GEAR_COMP);

            telemetry.addData("Tag Visible", tagVisible);

            if (tagVisible) {
                telemetry.addData("Tag ID", det.id);
                telemetry.addData("Tag center (x,y)", "%.1f, %.1f", cx, cy);
                telemetry.addData("Measured/Filtered", "%.1f / %.1f", measured, filtered);
                telemetry.addData("errorPx", "%.1f", errorPx);
                telemetry.addData("absErr", "%.1f", absErr);
                telemetry.addData("aimDir", aimDir);
                telemetry.addData("Flipped?", flippedThisLoop);
                telemetry.addData("In deadband?", inDeadband);

                if (det.ftcPose != null) {
                    telemetry.addData("Pose range(in)", "%.1f", det.ftcPose.range);
                    telemetry.addData("Pose bearing(deg)", "%.2f", det.ftcPose.bearing);
                    telemetry.addData("Pose yaw(deg)", "%.2f", det.ftcPose.yaw);
                }
            } else {
                telemetry.addData("Last seen (ms ago)", (lastSeenMs == 0) ? "never" : (now - lastSeenMs));
            }

            telemetry.addData("MoveDir", moveDirText);
            telemetry.addData("Turret before/after", "%.3f -> %.3f", turretBefore, turretPos1);
            telemetry.addData("Applied Δpos", "%.5f", appliedDelta);
            telemetry.addData("Loop dt (ms)", loopDtMs);

            telemetry.update();
        }

        if (visionPortal != null) visionPortal.close();
    }

    /* ===================== INIT ===================== */

    private void initHardware() {
        turretRotation1 = hardwareMap.get(Servo.class, "turret_rotation_1");
        turretRotation2 = hardwareMap.get(Servo.class, "turret_rotation_2");

        turretRotation1.setDirection(Servo.Direction.FORWARD);
        turretRotation2.setDirection(Servo.Direction.REVERSE);

        turretRotation1.setPosition(turretPos1);
        turretRotation2.setPosition(1.0 - turretPos1);
    }

    private void initVision() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "turretCam"), // KEEP NAME CONSISTENT
                aprilTag
        );
    }

    /* ===================== HELPERS ===================== */

    private AprilTagDetection getDetectionForId(int id) {
        if (aprilTag == null) return null;
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d != null && d.id == id) return d;
        }
        return null;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private boolean CAMERA_IS_PORtrait() {
        return CAMERA_IS_PORTRAIT;
    }
}
