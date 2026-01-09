package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo; // Import Servo
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection; // Import detection (placeholder)

import static org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.FieldPose.*;
import static org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.FieldPaths.*;
import org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.SwerveConstants;

@Autonomous(name = "Swerve Eventful Auto", group = "Swerve")
public class SwerveEventfulAuto extends OpMode {

    private Follower follower;
    private int pathIndex = 0;
    private PathAndEvent[] pathSequence;

    // --- New Hardware ---
    private Servo light; // The servo controlling the light color

    @Override
    public void init() {
        follower = SwerveConstants.createFollower(hardwareMap);
        FieldPaths.initializePaths(follower);
        follower.setStartingPose(redDepot);
        FieldEvent.initialize(hardwareMap);

        // --- Initialize Light ---
        // Make sure "light" matches your configuration name!
        light = hardwareMap.get(Servo.class, "light");
        light.setPosition(RGB.off); // Start with light off

        // Initialize Vision (Placeholder - Add your actual VisionPortal init here)
        // initAprilTag();

        pathSequence = new PathAndEvent[]{
                new PathAndEvent(Colorado, Event.SHOOT),
                new PathAndEvent(NewMexico, Event.INTAKE_ON),
                new PathAndEvent(NewMexicoB, Event.INTAKE_OFF),
                new PathAndEvent(Colorado2, Event.SHOOT),



        };

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        pathIndex = 0;
        if (pathIndex < pathSequence.length) {
            follower.followPath(pathSequence[pathIndex].path);
        }
    }

    @Override
    public void loop() {
        follower.update();

        // --- Light Logic ---
        // Check for AprilTag visibility
        boolean tagVisible = checkForAprilTag();

        if (tagVisible) {
            light.setPosition(RGB.green); // Tag seen -> Green
        } else {
            light.setPosition(RGB.blue);  // No tag -> Blue
        }

        // --- Path Logic ---
        if (!follower.isBusy()) {
            if (pathIndex < pathSequence.length) {
                boolean eventFinished = FieldEvent.perform(pathSequence[pathIndex].event);
                if (eventFinished) {
                    pathIndex++;
                    if (pathIndex < pathSequence.length) {
                        follower.followPath(pathSequence[pathIndex].path);
                    } else {
                        pathIndex = -1;
                    }
                }
            }
        }

        telemetry.addData("Light Color", tagVisible ? "Green" : "Blue");
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        light.setPosition(RGB.off); // Turn off light at end
    }

    // --- Helper Method for AprilTag ---
    private boolean checkForAprilTag() {
        // REPLACE THIS with your actual AprilTag detection logic!
        // Example:
        // List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        // return !currentDetections.isEmpty();

        return false; // Default to false (Blue light) if no vision code
    }
}