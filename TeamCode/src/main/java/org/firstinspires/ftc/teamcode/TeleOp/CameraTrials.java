package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo; // Added for lights

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Camera Target Indicator", group = "Concept")
public class CameraTrials extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;

    // --- Light Constants ---
    private Servo light;
    final double COLOR_RED    = 0.28;
    final double COLOR_YELLOW = 0.388;
    final double COLOR_GREEN  = 0.45;
    final double COLOR_BLUE   = 0.6;

    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // Initialize the light hardware
        light = hardwareMap.get(Servo.class, "light");
        light.setPosition(COLOR_BLUE); // Start with Blue (no tag yet)

        initAprilTag();

        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            updateLightAndTelemetry();
            telemetry.update();

            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }
            sleep(20);
        }
        visionPortal.close();
    }

    private void updateLightAndTelemetry() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        boolean foundTarget = false;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && !detection.metadata.name.contains("Obelisk")) {
                foundTarget = true;

                // 'ftcPose.bearing' is the angle to the tag relative to camera center
                double bearing = detection.ftcPose.bearing;

                // Logic: Red if within 5 degrees, Blue otherwise
                if (Math.abs(bearing) <= 5.0) {
                    light.setPosition(COLOR_RED);
                } else {
                    light.setPosition(COLOR_BLUE);
                }

                // Telemetry for debugging
                telemetry.addLine(String.format("\nTag: %s", detection.metadata.name));
                telemetry.addData("Bearing to Tag", "%.2f deg", bearing);
            }
        }

        // Default to Blue if no valid red targets are seen at all
        if (!foundTarget) {
            light.setPosition(COLOR_BLUE);
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }
}