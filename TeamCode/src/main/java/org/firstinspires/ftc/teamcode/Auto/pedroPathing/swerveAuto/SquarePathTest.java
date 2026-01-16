package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.SwerveConstants;

/**
 * Test OpMode to verify robot movement accuracy and Swerve Module Alignment.
 */
@Autonomous(name = "Square Path Test (X=Forward)", group = "Test")
public class SquarePathTest extends OpMode {

    private Follower follower;
    private int pathIndex = 0;
    private PathAndEvent[] pathSequence;

    private Timer pauseTimer = new Timer();
    private boolean isPausing = false;

    // Pose Definitions
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose corner1 = new Pose(24, 0, Math.toRadians(0));
    private final Pose corner2 = new Pose(24, 24, Math.toRadians(0));
    private final Pose corner3 = new Pose(0, 24, Math.toRadians(0));
    private final Pose corner4 = new Pose(0, 0, Math.toRadians(0));

    private final Pose[] targets = {corner1, corner2, corner3, corner4};
    private PathChain leg1Forward, leg2Left, leg3Back, leg4Right;

    @Override
    public void init() {
        follower = SwerveConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Build Paths
        leg1Forward = follower.pathBuilder()
                .addPath(new BezierLine(startPose, corner1))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        leg2Left = follower.pathBuilder()
                .addPath(new BezierLine(corner1, corner2))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        leg3Back = follower.pathBuilder()
                .addPath(new BezierLine(corner2, corner3))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        leg4Right = follower.pathBuilder()
                .addPath(new BezierLine(corner3, corner4))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        pathSequence = new PathAndEvent[]{
                new PathAndEvent(leg1Forward, Event.NULL),
                new PathAndEvent(leg2Left, Event.NULL),
                new PathAndEvent(leg3Back, Event.NULL),
                new PathAndEvent(leg4Right, Event.NULL)
        };

        telemetry.addData("Status", "Initialized.");
        telemetry.update();
    }

    @Override
    public void start() {
        pathIndex = 0;
        follower.followPath(pathSequence[pathIndex].path);
    }

    @Override
    public void loop() {
        follower.update();

        // Path Completion Logic
        if (!follower.isBusy()) {
            if (!isPausing) {
                isPausing = true;
                pauseTimer.resetTimer();
            }

            if (isPausing && pauseTimer.getElapsedTimeSeconds() > 5.0) {
                isPausing = false;
                pathIndex++;

                if (pathIndex < pathSequence.length) {
                    follower.followPath(pathSequence[pathIndex].path);
                } else {
                    pathIndex = -1;
                }
            }
        }

        // --- ENHANCED TELEMETRY ---

        telemetry.addLine("=== SWERVE DIAGNOSTICS ===");
        // This prints the string we formatted in SwerveDrivetrain.debugString()
        // Showing: Raw Angle (Servo) | Calc Angle (Robot Frame) | Target (Command)
        telemetry.addLine(follower.getDrivetrain().debugString());
        telemetry.update();

        telemetry.addLine("\n=== MOVEMENT STATUS ===");
        if (pathIndex != -1) {
            telemetry.addData("Leg", (pathIndex + 1) + "/4");
            //telemetry.addData("State", isPausing ? "PAUSED (" + (5 - (int)pauseTimer.getElapsedTimeSeconds()) + "s)" : "MOVING");

            Pose target = targets[pathIndex];
            // telemetry.addData("Target X", "%.2f", target.getX());
            // telemetry.addData("Actual X", "%.2f", follower.getPose().getX());
            // telemetry.addData("Target Y", "%.2f", target.getY());
            // telemetry.addData("Actual Y", "%.2f", follower.getPose().getY());
        } else {
            // telemetry.addData("Status", "DONE");
        }

        //telemetry.addLine("=== ODOMETRY RAW ===");
        //telemetry.addData("Heading (Deg)", "%.2f", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }
}