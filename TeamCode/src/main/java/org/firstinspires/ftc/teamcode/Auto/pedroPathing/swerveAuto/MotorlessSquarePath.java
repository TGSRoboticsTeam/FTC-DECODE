package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.SwerveConstants;
import org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.SwerveDrivetrain;

/**
 * Motorless Test OpMode.
 * LOGIC:
 * - Runs the full Pedro Pathing logic for the "Right-Hand Square".
 * - Actively steers the wheels (Axons) to point in the direction of travel.
 * - DISABLES drive motors so you can push the robot by hand.
 * USE CASE:
 * - Push the robot along the path.
 * - Verify wheels point in the correct direction for each leg.
 * - Verify odometry (X/Y) updates correctly as you push.
 * - Verify the code switches to the next leg when you reach the target.
 */
@Autonomous(name = "Motorless Square Path Test", group = "Test")
@Disabled
public class MotorlessSquarePath extends OpMode {

    private Follower follower;
    private int pathIndex = 0;
    private PathAndEvent[] pathSequence;

    private Timer pauseTimer = new Timer();
    private boolean isPausing = false;

    // Pose Definitions (Standard FTC: X=Forward, Y=Left)
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    // 1. Move Forward 24 inches (+X)
    private final Pose corner1 = new Pose(24, 0, Math.toRadians(0));

    // 2. Move Right 24 inches (-Y) - Robot is now at (24, -24)
    private final Pose corner2 = new Pose(24, -24, Math.toRadians(0));

    // 3. Move Backward 24 inches (-X) - Robot is now at (0, -24)
    private final Pose corner3 = new Pose(0, -24, Math.toRadians(0));

    // 4. Move Left 24 inches (+Y) - Robot is back at (0, 0)
    private final Pose corner4 = new Pose(0, 0, Math.toRadians(0));

    // Store targets in array for easy access by index
    private final Pose[] targets = {corner1, corner2, corner3, corner4};

    private PathChain leg1Forward, leg2Right, leg3Back, leg4Left;

    @Override
    public void init() {
        follower = SwerveConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Build Paths (Using Pose directly)
        leg1Forward = follower.pathBuilder()
                .addPath(new BezierLine(startPose, corner1))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        leg2Right = follower.pathBuilder()
                .addPath(new BezierLine(corner1, corner2))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        leg3Back = follower.pathBuilder()
                .addPath(new BezierLine(corner2, corner3))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        leg4Left = follower.pathBuilder()
                .addPath(new BezierLine(corner3, corner4))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .build();

        pathSequence = new PathAndEvent[]{
                new PathAndEvent(leg1Forward, Event.NULL),
                new PathAndEvent(leg2Right, Event.NULL),
                new PathAndEvent(leg3Back, Event.NULL),
                new PathAndEvent(leg4Left, Event.NULL)
        };

        telemetry.addData("Status", "Initialized. MOTORLESS MODE.");
        telemetry.addLine("Push robot to target to advance path.");
        telemetry.update();
    }

    @Override
    public void start() {
        pathIndex = 0;
        follower.followPath(pathSequence[pathIndex].path);
    }

    @Override
    public void loop() {
        // 1. Run the full Follower update (Calculates vectors, updates localizer)
        follower.update();

        // 2. CRITICAL: Override Drive Powers to 0 (Motorless Mode)
        if (follower.getDrivetrain() instanceof SwerveDrivetrain) {
            SwerveDrivetrain swerveDrive = (SwerveDrivetrain) follower.getDrivetrain();
            swerveDrive.stopDriveMotorsOnly();
        }

        // Path Completion Logic
        // Manual Distance Calculation since getDistance is not in Pose
        double distToTarget = getDistance(follower.getPose(), targets[pathIndex]);

        if (!follower.isBusy() || distToTarget < 2.0) { // 2 inch tolerance for hand pushing
            if (!isPausing) {
                isPausing = true;
                pauseTimer.resetTimer();
            }

            if (isPausing && pauseTimer.getElapsedTimeSeconds() > 2.0) { // 2 sec pause
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
        telemetry.addLine(follower.getDrivetrain().debugString());

        telemetry.addLine("\n=== MOVEMENT STATUS ===");
        if (pathIndex != -1) {
            telemetry.addData("Leg", (pathIndex + 1) + "/4");
            telemetry.addData("State", isPausing ? "PAUSED" : "MOVING (Push Me!)");

            Pose target = targets[pathIndex];
            telemetry.addData("Target X", "%.2f", target.getX());
            telemetry.addData("Actual X", "%.2f", follower.getPose().getX());

            telemetry.addData("Target Y", "%.2f", target.getY());
            telemetry.addData("Actual Y", "%.2f", follower.getPose().getY());

            telemetry.addData("Dist to Target", "%.2f", distToTarget);
        } else {
            telemetry.addData("Status", "DONE");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }

    /**
     * Manual calculation of Euclidean distance between two Poses.
     */
    private double getDistance(Pose p1, Pose p2) {
        double dx = p1.getX() - p2.getX();
        double dy = p1.getY() - p2.getY();
        return Math.hypot(dx, dy);
    }
}