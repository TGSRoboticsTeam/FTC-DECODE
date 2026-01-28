package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "geminiAttempt", group = "Tools")
public class geminiAttempt extends LinearOpMode {

    private Servo turret1, turret2;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Settings from your debug photo
    private double turretPos = 0.5;
    private final double FRAME_MID_Y = 240.0;
    private final double DEADBAND = 20.0;
    private final double AIM_STEP = 0.01;
    private final double AIM_SIGN = -1.0;

    private boolean autoAimEnabled = false;
    private boolean lbPrev = false;
    private int targetId = 24; // Red Goal

    @Override
    public void runOpMode() {
        // 1. Hardware Setup
        turret1 = hardwareMap.get(Servo.class, "turret_rotation_1");
        turret2 = hardwareMap.get(Servo.class, "turret_rotation_2");

        // Ensure directions are set for mirroring
        turret1.setDirection(Servo.Direction.FORWARD);
        turret2.setDirection(Servo.Direction.REVERSE);

        // 2. Vision Setup
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "turretCam"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("READY - USE GAMEPAD 1");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- INPUT HANDLING (GAMEPAD 1) ---
            if (gamepad1.left_bumper && !lbPrev) autoAimEnabled = !autoAimEnabled;
            lbPrev = gamepad1.left_bumper;

            if (gamepad1.dpad_left) targetId = 20;  // Blue
            if (gamepad1.dpad_right) targetId = 24; // Red

            // --- LOGIC ---
            String actionTaken = "IDLE";

            if (autoAimEnabled) {
                AprilTagDetection foundTag = null;
                List<AprilTagDetection> detections = aprilTag.getDetections();

                for (AprilTagDetection d : detections) {
                    if (d.id == targetId && d.center != null) {
                        foundTag = d;
                        break;
                    }
                }

                if (foundTag != null) {
                    double error = foundTag.center.y - FRAME_MID_Y;

                    if (Math.abs(error) > DEADBAND) {
                        double direction = (error > 0) ? 1.0 : -1.0;
                        turretPos += (AIM_SIGN * direction * AIM_STEP);
                        actionTaken = "STEPPING";
                    } else {
                        actionTaken = "LOCKED IN DEADBAND";
                    }
                } else {
                    actionTaken = "SEARCHING (NO TAG)";
                }
            } else {
                // MANUAL CONTROL (Gamepad 1 Right Stick X)
                double manual = -gamepad1.right_stick_x;
                if (Math.abs(manual) > 0.05) {
                    turretPos += manual * 0.02;
                    actionTaken = "MANUAL MOVE";
                }
            }

            // --- OUTPUT ---
            turretPos = Range.clip(turretPos, 0.0, 1.0);
            turret1.setPosition(turretPos);
            turret2.setPosition(turretPos); // Re-check if this should be turretPos or (1-turretPos) based on your Direction.REVERSE setting

            // --- DEBUG TELEMETRY ---
            telemetry.addData("AUTO-AIM", autoAimEnabled ? "ON" : "OFF (Manual)");
            telemetry.addData("Target ID", targetId);
            telemetry.addData("Turret Position", "%.3f", turretPos);
            telemetry.addData("Current Action", actionTaken);
            telemetry.addLine("\n--- DETAILED STATS ---");

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            if (!currentDetections.isEmpty()) {
                AprilTagDetection d = currentDetections.get(0);
                telemetry.addData("Visible Tag ID", d.id);
                telemetry.addData("Raw Y Center", "%.2f", d.center.y);
                telemetry.addData("Calculated Error", "%.2f", d.center.y - FRAME_MID_Y);
            } else {
                telemetry.addLine("NO TAGS IN VIEW");
            }

            telemetry.update();
        }
    }
}