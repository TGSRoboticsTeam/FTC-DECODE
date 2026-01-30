package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Complex Swerve Auto", group = "Swerve")
public class ComplexPathAuto extends OpMode {
    private Follower follower;
    private int pathIndex = 0;
    private PathAndEvent[] sequence;

    @Override
    public void init() {
        follower = SwerveConstants.createFollower(hardwareMap);
        FieldEvent.initialize(hardwareMap);

        // Step 1: Start at 0,0 Heading 0
        Pose startPose = new Pose(0, 0, 0);
        follower.setStartingPose(startPose);

        // Build the sequence
        sequence = new PathAndEvent[]{
                // 2. Launch 3 times
                new PathAndEvent(null, Event.LAUNCH_THREE),

                // 3. Align pods left/right & Pause 1s
                new PathAndEvent(null, Event.ALIGN_LEFT_RIGHT),
                new PathAndEvent(null, Event.PAUSE_LONG),

                // 4. Move 51 inches Right (Y is left in standard Pedro, so -51 is right)
                new PathAndEvent(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(0,0,0), new Pose(0, -51, 0)))
                        .setConstantHeadingInterpolation(0).build(), Event.PAUSE_SHORT),

                // 5. Intake On & Pause
                new PathAndEvent(null, Event.INTAKE_ON),
                new PathAndEvent(null, Event.PAUSE_SHORT),

                // 6. Align pods Forward/Back & Pause 1s
                new PathAndEvent(null, Event.ALIGN_FORWARD_BACK),
                new PathAndEvent(null, Event.PAUSE_LONG),

                // 7. Move forward 30 inches
                new PathAndEvent(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(0, -51, 0), new Pose(30, -51, 0)))
                        .setConstantHeadingInterpolation(0).build(), Event.PAUSE_SHORT),

                // 8. Intake Off & Move Back 30
                new PathAndEvent(null, Event.INTAKE_OFF),
                new PathAndEvent(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(30, -51, 0), new Pose(0, -51, 0)))
                        .setConstantHeadingInterpolation(0).build(), Event.ALIGN_LEFT_RIGHT),

                // 9. Move Left 51 (Back to 0,0) & Launch again
                new PathAndEvent(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(0, -51, 0), new Pose(0, 0, 0)))
                        .setConstantHeadingInterpolation(0).build(), Event.PAUSE_LONG),
                new PathAndEvent(null, Event.LAUNCH_THREE),

                // 10. Diagonal movement (45 degrees)
                new PathAndEvent(null, Event.ALIGN_DIAGONAL),
                new PathAndEvent(follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(0,0,0), new Pose(35.35, 35.35, Math.toRadians(45))))
                        .setLinearHeadingInterpolation(0, Math.toRadians(45)).build(), Event.NULL)
        };
    }

    @Override
    public void loop() {
        follower.update();
        if (pathIndex >= 0 && pathIndex < sequence.length) {
            PathAndEvent current = sequence[pathIndex];

            // If there is a path, follow it. If no path, or path is done, do event.
            if (current.path != null && !follower.isBusy() ) {
                follower.followPath(current.path);
            }

            if (current.path == null || (!follower.isBusy())) {
                if (FieldEvent.perform(current.event)) {
                    pathIndex++;
                }
            }
        }
    }
}