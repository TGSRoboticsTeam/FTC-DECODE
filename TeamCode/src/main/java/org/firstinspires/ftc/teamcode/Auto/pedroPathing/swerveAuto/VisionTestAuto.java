package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

// Import your SwerveConstants
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.SwerveConstants;

// Import the RGB class for colors
import org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.RGB;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Vision Test Auto (No Move)", group = "Test")
@Disabled
public class VisionTestAuto extends OpMode {

    private Follower follower;
    private Servo light;
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;


    // Enum to represent the detected color state
    private enum TagColor {
        RED,
        BLUE,
        PURPLE,
        GREEN,
        WHITE,
        NONE
    }

    @Override
    public void init() {
        // Initialize the follower with SwerveConstants just like the real auto
        // This ensures all hardware is configured correctly, even if we don't move.
        follower = SwerveConstants.createFollower(hardwareMap);

        // We set the starting pose, but we won't be moving from it.
        follower.setStartingPose(FieldPose.redDepot);

        // Initialize the light servo
        // Make sure the name "light" matches your Robot Configuration on the Driver Station
        light = hardwareMap.get(Servo.class, "light");
        light.setPosition(RGB.off); // Start with the light off

        // Initialize your Vision System here!
        initAprilTag();

        telemetry.addData("Status", "Initialized. Ready for Vision Test.");
        telemetry.update();
    }

    @Override
    public void start() {
        // We do NOT start any paths here. The robot stays still.
        telemetry.addData("Status", "Started. Watching for tags...");
    }

    @Override
    public void loop() {
        // Ideally, we still update the follower so odometry/localization is processed
        // even if we aren't moving. This is good for debugging your pod readings.
        follower.update();

        // 1. Get the current detection status
        TagColor detectedColor = getDetectedTagColor();

        // 2. Set the light based on the detection
        telemetry.addData("Detected", light.getPosition());

        switch (detectedColor) {
            case RED:
                light.setPosition(RGB.red);
                telemetry.addData("Tag Seen", "RED Target");
                break;
            case GREEN:
                light.setPosition(RGB.green);
                telemetry.addData("Tag Seen", "GreenPurplePurple");
                break;
            case PURPLE:
                light.setPosition(RGB.violet);
                telemetry.addData("Tag Seen", "PurplePurpleGreen");
                break;
            case BLUE:
                light.setPosition(RGB.blue);
                telemetry.addData("Tag Seen", "BLUE Target");
                break;
            case WHITE:
                light.setPosition(RGB.white);
                telemetry.addData("Tag Seen", "PurpleGreenPurple");
                break;
            case NONE:
            default:
                light.setPosition(RGB.orange);
                telemetry.addData("Tag Seen", "NONE");
                break;
        }

        // Telemetry to show we are alive
        telemetry.addData("Light Position", light.getPosition());
        telemetry.addData("Pose", follower.getPose().toString());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Turn off the light when the OpMode stops
        light.setPosition(RGB.off);
    }

    /**
     * Placeholder method for your Vision Logic.
     * You must replace the logic inside to check your actual AprilTag processor.
     */
    private TagColor getDetectedTagColor() {
        // --- YOUR VISION CODE GOES HERE ---

        // Example Logic (Pseudo-code):
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        telemetry.addData("tags seen: ", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                // Check tag ID or metadata name to determine color
                double range = detection.ftcPose.range;
                double yaw = Math.toRadians(detection.ftcPose.yaw);
                double pitch = Math.toRadians(detection.ftcPose.pitch);
                double bearing = Math.toRadians(detection.ftcPose.bearing);

                telemetry.addData("range: ", range);
                telemetry.addData("yaw: ", yaw);
                telemetry.addData("pitch: ", pitch);
                telemetry.addData("bearing: ", bearing);

                int id = detection.id;
                telemetry.addData("id seen: ", id);

                if (id == 20) { // Red Alliance Tags
                    return TagColor.BLUE;
                } else if (id == 24) { // Blue Alliance Tags
                    return TagColor.RED;
                } else if (id == 21) { // Blue Alliance Tags
                    return TagColor.GREEN;
                } else if (id == 22) { // Blue Alliance Tags
                    return TagColor.WHITE;
                } else if (id == 23) {
                    return TagColor.PURPLE;// Blue Alliance Tags
                }
            }
         }

        // If no relevant tag is found:
        return TagColor.NONE;
    }



    private void initAprilTag() {
        // Create the AprilTag processor and VisionPortal here
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTagProcessor);
    }

    // Optional: Add your vision initialization method here if it's local to this class
    // private void initAprilTag() { ... }
}