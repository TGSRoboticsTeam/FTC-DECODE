package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Disabled
@TeleOp(name = "TurretAutoAim_SimplePixel_DEBUG", group = "Tools")
public class SwerveTurretAutoAimTest extends LinearOpMode {

    /* ===================== TURRET SERVOS ===================== */
    private Servo turretRotation1, turretRotation2;

    private static final double MIN_TURRET = 0.0;
    private static final double MAX_TURRET = 1.0;

    // step each loop when tag is off-center
    private static final double AIM_STEP = 0.01;

    // how close to center counts as "centered"
    private static final double PIXEL_DEADBAND = 20.0;

    // YOU SAID YOU FLIPPED SIGN ON YOUR ROBOT
    // If it moves the wrong direction, flip this again.
    private static final double AIM_SIGN = -1.0;

    private double turretPos = 0.5;

    /* ===================== CAMERA / APRILTAG ===================== */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // CenterStage goal IDs (change if your game uses different IDs)
    private static final int TAG_BLUE_GOAL = 20;
    private static final int TAG_RED_GOAL  = 24;

    private int targetId = TAG_RED_GOAL;

    private boolean autoAimEnabled = true;
    private boolean togglePrev = false;

    // Camera mounted vertically: use tag.center.y
    // Midpoint assumes 640x480 (midpoint Y = 240). Change if stream size differs.
    private static final double FRAME_MID_Y = 240.0;

    /* ===================== DEBUG / TIMING ===================== */
    private long lastSeenMs = 0;

    private long prevLoopMs = 0;
    private long loopDtMs = 0;

    @Override
    public void runOpMode() {

        initHardware();
        initVision();

        telemetry.addLine("TurretAutoAim_SimplePixel_DEBUG ready");
        telemetry.addLine("GP2: LB toggle autoaim | dpad_left=BLUE | dpad_right=RED | RSX manual when autoaim OFF");
        telemetry.addLine("Camera vertical: using tag.center.y for aiming axis");
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

            double cy = Double.NaN;
            double errorPx = Double.NaN;

            boolean inDeadband = false;
            String moveDirText = "NONE";

            double appliedDelta = 0.0;
            double turretBefore = turretPos;

            /* ===================== AIM LOGIC ===================== */
            if (!autoAimEnabled) {
                // Manual turret control when autoaim OFF
                double manual = -gamepad2.right_stick_x;
                if (Math.abs(manual) < 0.05) manual = 0;

                appliedDelta = manual * 0.01;
                turretPos = clamp(turretPos + appliedDelta, MIN_TURRET, MAX_TURRET);
                moveDirText = (manual == 0) ? "MANUAL HOLD" : "MANUAL";

            } else {
                // Auto aim ON
                if (tagVisible) {
                    // ALWAYS compute error if detection exists (fixes your NaN issue)
                    cy = det.center.y;
                    errorPx = cy - FRAME_MID_Y;     // + means "below center"
                    inDeadband = Math.abs(errorPx) <= PIXEL_DEADBAND;

                    lastSeenMs = now;

                    if (!inDeadband) {
                        double dir = (errorPx > 0) ? +1.0 : -1.0;
                        moveDirText = (dir > 0) ? "POS" : "NEG";

                        appliedDelta = AIM_SIGN * dir * AIM_STEP;
                        turretPos = clamp(turretPos + appliedDelta, MIN_TURRET, MAX_TURRET);
                    } else {
                        moveDirText = "DEADBAND (HOLD)";
                        appliedDelta = 0.0;
                    }
                } else {
                    moveDirText = "NO TAG (HOLD)";
                    appliedDelta = 0.0;
                }
            }

            /* ===================== APPLY SERVOS ===================== */
            turretRotation1.setPosition(turretPos);
            turretRotation2.setPosition(1.0 - turretPos);

            /* ===================== TELEMETRY (EVERYTHING) ===================== */
            telemetry.addData("AutoAim", autoAimEnabled ? "ON" : "OFF");
            telemetry.addData("Target", targetId == TAG_RED_GOAL ? "RED (24)" : "BLUE (20)");

            telemetry.addData("AIM_SIGN", "%.1f", AIM_SIGN);
            telemetry.addData("AIM_STEP", "%.3f", AIM_STEP);
            telemetry.addData("DEADBAND(px)", "%.1f", PIXEL_DEADBAND);
            telemetry.addData("FRAME_MID_Y", "%.1f", FRAME_MID_Y);

            telemetry.addData("Tag Visible", tagVisible);

            if (tagVisible) {
                telemetry.addData("Tag ID", det.id);
                telemetry.addData("Tag center (x,y)", "%.1f, %.1f", det.center.x, det.center.y);
                telemetry.addData("Axis used", "center.y (camera vertical)");
                telemetry.addData("errorPx = y-mid", "%.1f", errorPx);
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
            telemetry.addData("Turret before/after", "%.3f -> %.3f", turretBefore, turretPos);
            telemetry.addData("Applied Δpos", "%.4f", appliedDelta);
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

        turretRotation1.setPosition(turretPos);
        turretRotation2.setPosition(1.0 - turretPos);
    }

    private void initVision() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "turretCam"),
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
}
