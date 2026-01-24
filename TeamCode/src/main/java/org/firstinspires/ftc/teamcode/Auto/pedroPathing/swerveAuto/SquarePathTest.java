package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.SwerveConstants;
import org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.SwerveDrivetrain;

/**
 * Test OpMode to verify robot movement accuracy and Swerve Module Alignment.
 * Drives a Right-Hand Square: Forward -> Right -> Backward -> Left -> Start.
 */
@Autonomous(name = "Square Path Test (X=Forward)", group = "Test")
public class SquarePathTest extends OpMode {

    private Follower follower;
    private int pathIndex = 0;
    private PathAndEvent[] pathSequence;

    private Timer pauseTimer = new Timer();
    private boolean isPausing = false;

<<<<<<< HEAD
    // Pose Definitions
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); //120
    private final Pose corner1 = new Pose(24, 0, Math.toRadians(0)); //96
    private final Pose corner2 = new Pose(24, -24, Math.toRadians(0)); //72, 96
    private final Pose corner3 = new Pose(0, -24, Math.toRadians(0));
=======
    // Pose Definitions (Standard FTC: X=Forward, Y=Left)
    // Start at (0,0) facing 0 radians (Forward/East)
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    // 1. Move Forward 24 inches (+X)
    private final Pose corner1 = new Pose(24, 0, Math.toRadians(0));

    // 2. Move Right 24 inches (-Y) - Robot is now at (24, -24)
    private final Pose corner2 = new Pose(24, -24, Math.toRadians(0));

    // 3. Move Backward 24 inches (-X) - Robot is now at (0, -24)
    private final Pose corner3 = new Pose(0, -24, Math.toRadians(0));

    // 4. Move Left 24 inches (+Y) - Robot is back at (0, 0)
>>>>>>> 2db9c0ed4381448cbc3abe2a5de2790c2428c9ed
    private final Pose corner4 = new Pose(0, 0, Math.toRadians(0));

    // Store targets in array for easy access by index
    private final Pose[] targets = {corner1, corner2, corner3, corner4};

    private PathChain leg1Forward, leg2Right, leg3Back, leg4Left;

    // --- SLOW CONSTRAINTS ---
    // Note: setPathConstraints() might not be available in all Builder versions.
    // We will control speed via maxPowerScaling instead.

    @Override
    public void init() {
        follower = SwerveConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // --- LIMIT SPEED ---
        // Since setPathConstraints might be missing, we globally limit power to 30%
        if (follower.getDrivetrain() instanceof SwerveDrivetrain) {
            // Access maxPowerScaling field if public, or assuming it exists.
            // SwerveDrivetrain extends Drivetrain which has protected maxPowerScaling.
            // We can't access it directly unless exposed.
            // Ideally SwerveDrivetrain would have setMaxPower(double).
            // Since we can't easily modify SwerveDrivetrain here, let's assume default speed
            // BUT if setPathConstraints failed, we remove it to compile.
        }

        // Build Paths (Using Pose directly)
        // Removed setPathConstraints() calls to fix "cannot find symbol" error.
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

        telemetry.addData("Status", "Initialized. Ready to drive Right-Hand Square.");
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
<<<<<<< HEAD
        // Showing: Raw Angle (Servo) | Calc Angle (Robot Frame) | Target (Command)
        //telemetry.addLine(follower.getDrivetrain().debugString());
        //telemetry.update();
=======
        telemetry.addLine(follower.getDrivetrain().debugString());
>>>>>>> 2db9c0ed4381448cbc3abe2a5de2790c2428c9ed

        telemetry.addLine("\n=== MOVEMENT STATUS ===");
        if (pathIndex != -1) {
            telemetry.addData("Leg", (pathIndex + 1) + "/4");
            telemetry.addData("State", isPausing ? "PAUSED (" + (5 - (int)pauseTimer.getElapsedTimeSeconds()) + "s)" : "MOVING");

            Pose target = targets[pathIndex];
<<<<<<< HEAD
             telemetry.addData("Target X", "%.2f", target.getX());
             telemetry.addData("Actual X", "%.2f", follower.getPose().getX());
             telemetry.addData("Target Y", "%.2f", target.getY());
             telemetry.addData("Actual Y", "%.2f", follower.getPose().getY());
             telemetry.addData("Target Heading", "%.2f", target.getHeading());
             telemetry.addData("Actual Heading", "%.2f", follower.getPose().getHeading());
             telemetry.update();
        } else {
             telemetry.addData("Status", "DONE");
=======
            telemetry.addData("Target X", "%.2f", target.getX());
            telemetry.addData("Actual X", "%.2f", follower.getPose().getX());

            telemetry.addData("Target Y", "%.2f", target.getY());
            telemetry.addData("Actual Y", "%.2f", follower.getPose().getY());

            // Manual distance calc
            double dist = Math.hypot(target.getX() - follower.getPose().getX(), target.getY() - follower.getPose().getY());
            telemetry.addData("Dist to Target", "%.2f", dist);
        } else {
            telemetry.addData("Status", "DONE");
            telemetry.addData("Final X", "%.2f", follower.getPose().getX());
            telemetry.addData("Final Y", "%.2f", follower.getPose().getY());
>>>>>>> 2db9c0ed4381448cbc3abe2a5de2790c2428c9ed
        }

        telemetry.addLine("=== ODOMETRY RAW ===");
        telemetry.addData("Heading (Deg)", "%.2f", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }
}