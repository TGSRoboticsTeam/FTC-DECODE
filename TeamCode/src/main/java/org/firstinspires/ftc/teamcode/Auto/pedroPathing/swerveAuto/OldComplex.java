package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Custom Auto Routine: 3-Shot Cycle -> Strafe -> Intake -> Diagonal.
 * All movements relative to start 0,0 heading 0.
 */
@Autonomous(name = "Complex Path Auto", group = "Swerve")
@Disabled
public class OldComplex extends OpMode {

    private Follower follower;
    private int pathIndex = 0;
    private PathAndEvent[] sequence;

    @Override
    public void init() {
        follower = SwerveConstants.createFollower(hardwareMap);
        FieldEvent.initialize(hardwareMap);

        // 1. Set Starting Position: 0,0 Heading 0
        Pose startPose = new Pose(0, 0, 0);
        follower.setStartingPose(startPose);

        // Define specific positions based on your requirements
        Pose posStart     = new Pose(0, 0, 0);
        Pose posRight51   = new Pose(0, -51, 0);  // Y is left, so -51 is Right
        Pose posForward30 = new Pose(30, -51, 0); // Forward 30 from the strafed position
        Pose posBack30    = new Pose(0, -51, 0);  // Back to the strafed line
        Pose posDiagonal  = new Pose(35.35, 35.35, Math.toRadians(45)); // 50" at 45 deg

        // Pre-build the PathChains
        PathChain moveRight = follower.pathBuilder()
                .addPath(new BezierLine(posStart, posRight51))
                .setConstantHeadingInterpolation(0).build();

        PathChain moveForward = follower.pathBuilder()
                .addPath(new BezierLine(posRight51, posForward30))
                .setConstantHeadingInterpolation(0).build();

        PathChain moveBack = follower.pathBuilder()
                .addPath(new BezierLine(posForward30, posBack30))
                .setConstantHeadingInterpolation(0).build();

        PathChain moveLeftReturn = follower.pathBuilder()
                .addPath(new BezierLine(posBack30, posStart))
                .setConstantHeadingInterpolation(0).build();

        PathChain moveDiagonal = follower.pathBuilder()
                .addPath(new BezierLine(posStart, posDiagonal))
                .setLinearHeadingInterpolation(0, Math.toRadians(45)).build();

        // Build the sequence array
        sequence = new PathAndEvent[]{
                // --- Segment 1: Launch and Strafe ---
                new PathAndEvent(null, Event.LAUNCH_START_THREE),    // Launch 3 times
                new PathAndEvent(null, Event.ALIGN_LEFT_RIGHT),      // Rotate pods left/right
                new PathAndEvent(null, Event.PAUSE_1SEC),            // Pause 1s
                new PathAndEvent(moveRight, Event.PAUSE_05SEC),      // Move 51" Right + 0.5s pause

                // --- Segment 2: Intake Cycle ---
                new PathAndEvent(null, Event.INTAKE_ON),             // Turn on intake
                new PathAndEvent(null, Event.PAUSE_05SEC),           // Pause 0.5s
                new PathAndEvent(null, Event.ALIGN_FORWARD_BACK),    // Rotate pods forward/back
                new PathAndEvent(null, Event.PAUSE_1SEC),            // Pause 1s
                new PathAndEvent(moveForward, Event.PAUSE_05SEC),    // Move forward 30" + 0.5s pause
                new PathAndEvent(null, Event.INTAKE_OFF),            // Turn off intake
                new PathAndEvent(moveBack, Event.ALIGN_LEFT_RIGHT),  // Move back 30" + re-align pods
                new PathAndEvent(null, Event.PAUSE_1SEC),            // Pause 1s
                new PathAndEvent(moveLeftReturn, Event.PAUSE_1SEC),  // Move left 51" back to origin + 1s pause

                // --- Segment 3: Final Launch and Diagonal ---
                new PathAndEvent(null, Event.LAUNCH_START_THREE),    // Launch 3 times again
                new PathAndEvent(null, Event.PAUSE_1SEC),            // Pause 1s
                new PathAndEvent(null, Event.ALIGN_DIAGONAL),        // Turn pods to 45 degrees
                new PathAndEvent(null, Event.PAUSE_1SEC),            // Pause 1s
                new PathAndEvent(moveDiagonal, Event.NULL)           // Drive 50" diagonal
        };
    }

    @Override
    public void loop() {
        follower.update();

        if (pathIndex >= 0 && pathIndex < sequence.length) {
            PathAndEvent currentStep = sequence[pathIndex];

            // 1. Path Following Logic
            // Start the path if it exists and we haven't started it yet
            if (currentStep.path != null && !follower.isBusy()) {
                follower.followPath(currentStep.path);
            }

            // 2. Event Execution Logic
            // If there is no path, or the follower has finished the path, run the event
            if (currentStep.path == null || !follower.isBusy()) {
                // FieldEvent.perform must return true for the sequence to advance
                boolean eventDone = FieldEvent.perform(currentStep.event);

                if (eventDone) {
                    pathIndex++; // Advance only when the event (firing/pausing) is complete
                }
            }
        } else {
            telemetry.addData("Status", "Auto Sequence Complete");
        }

        // Diagnostics
        telemetry.addData("Path Index", pathIndex);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }
}