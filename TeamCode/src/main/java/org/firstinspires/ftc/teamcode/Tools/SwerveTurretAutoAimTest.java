package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "SwerveTurretAutoAimTest", group = "TeleOp")
public class SwerveTurretAutoAimTest extends LinearOpMode {

    // --- HARDWARE ---
    private Servo turretRotation1, turretRotation2;

    // --- TURRET SERVO LIMITS (same idea as your code) ---
    private static final double MIN_TURRET_ROTATION = 0.0;
    private static final double MAX_TURRET_ROTATION = 1.0;

    // Start centered
    private double currentTurretRotation = (MIN_TURRET_ROTATION + MAX_TURRET_ROTATION) / 2.0;

    // --- YOUR GEAR RATIO NOTE ---
    // "For every 1 rotation of the gears we have .7885 rotation of the turret"
    // If the servo-side gear rotation produces turret rotation at 0.7885:1,
    // then to get the same turret motion you need to command servo a bit more:
    private static final double TURRET_PER_GEAR = 0.7885;
    private static final double GEAR_TO_TURRET_COMP = 1.0 / TURRET_PER_GEAR; // ~1.268

    // --- VISION ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // DECODE goal tag IDs (manual)
    private static final int TAG_ID_BLUE_GOAL = 20;
    private static final int TAG_ID_RED_GOAL  = 24;

    // --- CAMERA ORIENTATION ---
    // If your camera is mounted "portrait" (rotated 90 degrees), set true.
    // Then we use centerY instead of centerX for left/right aiming.
    private static final boolean CAMERA_IS_PORTRAIT = true;

    // --- AIM TUNING ---
    // Very small steps, as requested. Start here, then adjust.
    private static final double KP = 0.010;            // proportional gain on normalized error
    private static final double MAX_STEP = 0.0030;      // absolute max step per loop (tiny)
    private static final double DEADBAND_NORM = 0.02;   // ~2% of half-frame

    // Choose which goal to aim at (toggle in TeleOp)
    private int targetTagId = TAG_ID_RED_GOAL;

    @Override
    public void runOpMode() {
        initializeHardware();
        initVision();

        telemetry.addLine("Turret Auto Aim Ready");
        telemetry.addLine("Gamepad:");
        telemetry.addLine("  Y = aim RED goal (ID 24)");
        telemetry.addLine("  X = aim BLUE goal (ID 20)");
        telemetry.addLine("  Right stick X = manual trim (overrides auto while moved)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Target select
            if (gamepad1.y) targetTagId = TAG_ID_RED_GOAL;
            if (gamepad1.x) targetTagId = TAG_ID_BLUE_GOAL;

            // Manual override (trim)
            double manual = -gamepad1.right_stick_x;
            boolean manualActive = Math.abs(manual) > 0.08;

            AprilTagDetection target = findTargetDetection(targetTagId);

            if (manualActive) {
                // Manual nudging (still small)
                double manualStep = manual * 0.004; // small manual rate
                applyTurretDelta(manualStep);
            } else if (target != null) {
                // AUTO AIM: center the tag in the image with tiny steps.
                // Use X for normal landscape, or Y if camera is rotated portrait.
                double measured = CAMERA_IS_PORTRAIT ? target.center.y : target.center.x;

                // These are pixel-ish coordinates in the camera frame.
                // We need a "frame center" value. The SDK doesn’t hand us width/height directly here,
                // so we estimate by using the tag's observed center bounds:
                // Best practice: set these to your stream resolution if you know it.
                // If you don't, this works "well enough" for basic centering:
                double assumedFrameSize = CAMERA_IS_PORTRAIT ? 720.0 : 1280.0; // adjust if needed
                double frameCenter = assumedFrameSize / 2.0;

                double errorPixels = measured - frameCenter;
                double halfFrame = assumedFrameSize / 2.0;
                double errorNorm = errorPixels / halfFrame; // -1..+1-ish

                // Deadband so it stops hunting when basically centered
                if (Math.abs(errorNorm) > DEADBAND_NORM) {
                    // Proportional step, clipped very small
                    double step = clip(errorNorm * KP, -MAX_STEP, MAX_STEP);

                    // Convert “desired turret movement” to “servo command” using your ratio note
                    step *= GEAR_TO_TURRET_COMP;

                    // Sign: if tag is right of center, rotate turret right (you may invert if needed)
                    applyTurretDelta(step);
                }
            }

            // Set the mirrored servos
            turretRotation1.setPosition(currentTurretRotation);
            turretRotation2.setPosition(1.0 - currentTurretRotation);

            // Telemetry
            telemetry.addData("Target Tag", "%s (ID %d)",
                    (targetTagId == TAG_ID_RED_GOAL ? "RED GOAL" : "BLUE GOAL"),
                    targetTagId);

            if (target != null) {
                telemetry.addData("Tag Seen", "YES");
                telemetry.addData("Tag Center (x,y)", "%.1f, %.1f", target.center.x, target.center.y);
            } else {
                telemetry.addData("Tag Seen", "NO");
            }

            telemetry.addData("Turret Pos", "%.4f", currentTurretRotation);
            telemetry.addData("Camera Portrait?", CAMERA_IS_PORTRAIT);
            telemetry.update();
        }

        // Cleanup
        if (visionPortal != null) visionPortal.close();
    }

    // --- HELPERS ---

    private void initializeHardware() {
        turretRotation1 = hardwareMap.get(Servo.class, "turret_rotation_1");
        turretRotation2 = hardwareMap.get(Servo.class, "turret_rotation_2");

        turretRotation1.setDirection(Servo.Direction.FORWARD);
        turretRotation2.setDirection(Servo.Direction.REVERSE);

        turretRotation1.setPosition(currentTurretRotation);
        turretRotation2.setPosition(1.0 - currentTurretRotation);
    }

    private void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                // If you know your tag size, you can set it for pose data; not required just to center.
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "turretCam"))
                .addProcessor(aprilTag)
                .build();
    }

    private AprilTagDetection findTargetDetection(int wantedId) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty()) return null;

        // Prefer exact match; otherwise return null (keeps behavior simple/predictable)
        for (AprilTagDetection d : detections) {
            if (d != null && d.id == wantedId) return d;
        }
        return null;
    }

    private void applyTurretDelta(double delta) {
        currentTurretRotation += delta;
        currentTurretRotation = clip(currentTurretRotation, MIN_TURRET_ROTATION, MAX_TURRET_ROTATION);
    }

    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
