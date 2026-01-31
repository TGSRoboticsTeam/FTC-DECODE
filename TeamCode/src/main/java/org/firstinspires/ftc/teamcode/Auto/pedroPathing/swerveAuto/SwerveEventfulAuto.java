package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.FieldPose.*;
import static org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.FieldPaths.*;
import org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.SwerveConstants;

@Autonomous(name = "Swerve Eventful Auto", group = "Swerve")
public class SwerveEventfulAuto extends OpMode {

    private Follower follower;
    private int pathIndex = 0;
    private PathAndEvent[] pathSequence;
    private Servo light;

    @Override
    public void init() {
        follower = SwerveConstants.createFollower(hardwareMap);
        FieldPaths.initializePaths(follower);
        follower.setStartingPose(redDepot);
        FieldEvent.initialize(hardwareMap);

        // Initialize Light
        light = hardwareMap.get(Servo.class, "light");
        light.setPosition(RGB.off);

        pathSequence = new PathAndEvent[]{
                new PathAndEvent(Colorado, Event.SHOOT),
                new PathAndEvent(NewMexico, Event.NULL)
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

        // --- Logic Fix ---
        // Only run path logic if we have a valid index (0 to length-1)
        if (pathIndex >= 0 && pathIndex < pathSequence.length) {

            // Check if the robot is done moving along the current path
            if (!follower.isBusy()) {

                // Execute the event associated with this path step
                boolean eventFinished = FieldEvent.perform(pathSequence[pathIndex].event);

                // Only proceed if the event is totally finished
                if (eventFinished) {
                    pathIndex++; // Advance to next step

                    // Check if there is a next path to follow
                    if (pathIndex < pathSequence.length) {
                        follower.followPath(pathSequence[pathIndex].path);
                    } else {
                        // Sequence Complete
                        pathIndex = -1;
                    }
                }
            }
        } else {
            // Path sequence is complete or invalid.
            // We can add "Holding" logic here if desired.
            if (pathIndex == -1) {
                telemetry.addData("Status", "Auto Sequence Complete");
            }
        }

        telemetry.addData("Path Index", pathIndex);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
        light.setPosition(RGB.off);
    }
}