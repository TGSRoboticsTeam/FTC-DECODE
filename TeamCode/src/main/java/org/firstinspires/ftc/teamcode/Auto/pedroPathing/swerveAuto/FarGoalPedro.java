package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// Pedro v2.1+ Imports
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

// Import your custom constants factory
import static org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.SwerveConstants.*;

@Autonomous(name = "Pedro Swerve: Far Goal Final", group = "Swerve")
public class FarGoalPedro extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState = 0;
    final double TRIGGER_FIRE = 0.0;
    final double TRIGGER_HOME = 0.225;

    // --- Mechanism Hardware ---
    private DcMotor leftFly, rightFly, frontIntake, backIntake;
    private Servo pusher, adjuster;

    // --- Path Definitions ---
    private PathChain driveToRight, driveToIntake, returnToScore,driveTo2ndIntake,returnTo2ndScore;
    private final Pose startPose = new Pose(0, 0, 0);

    public void buildPaths() {
        // Using the v2.1 Pose-based BezierLine
        driveToRight = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(0, 0, 0), new Pose(0, -51, 0)))
                .setConstantHeadingInterpolation(0)
                .build();

        driveToIntake = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(0, -51, 0), new Pose(30, -51, 0)))
                .setConstantHeadingInterpolation(0)
                .build();

        returnToScore = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(30, -51, 0), new Pose(0, 0, 0)))
                .setConstantHeadingInterpolation(0)
                .build();

        driveTo2ndIntake = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(0, 0, 0), new Pose(0, -75, 0)))
                .setConstantHeadingInterpolation(0)
                .build();

        returnTo2ndScore = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(0, -75, 0), new Pose(30, 75, 0)))
                .setConstantHeadingInterpolation(0)
                .build();

    }

    @Override
    public void runOpMode() {
        // Utilize your factory method to create the Follower
        // This links your SwerveDrivetrain and SwerveLocalizer automatically
        follower = SwerveConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        pathTimer = new Timer();
        initMechanisms();
        buildPaths();

        telemetry.addData("Status", "Swerve Follower Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        setPathState(0);

        while (opModeIsActive() && !isStopRequested()) {
            follower.update();
            autonomousPathUpdate();

            // Push swerve-specific telemetry to Dashboard
            telemetry.addData("Path State", pathState);
            telemetry.addData("Follower Busy", follower.isBusy());
            //get position of the robot from the localizer
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", follower.getPose().getHeading());
            telemetry.update();
            drawTelemetry();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start with Launch
              //  adjuster.setPosition(.118);
               // leftFly.setPower(-0.78);
               // rightFly.setPower(0.78);
               // pathTimer.resetTimer();
              //  if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                  //  fireThreeTimes();
                    follower.followPath(driveToRight);
                    setPathState(1);
              //  }
                break;

            case 1: // Moving Right 51 inches
                if (!follower.isBusy()) {
                    frontIntake.setPower(1.0);
                    follower.followPath(driveToIntake);
                    setPathState(2);
                }
                break;

            case 2: // Intaking Forward 30 inches
                if (!follower.isBusy()) {
                    frontIntake.setPower(0);
                    follower.followPath(returnToScore);
                    setPathState(3);
                }
                break;

            case 3: // Return & Shutdown
                if (!follower.isBusy()) {
                    //fireThreeTimes();
                    follower.followPath(driveTo2ndIntake);
                    setPathState(4);
                }
                break;

            case 4: // Return & Shutdown
                if (!follower.isBusy()) {
                    follower.followPath(returnTo2ndScore);
                    setPathState(5);
                }
                break;

            case 5: // Return & Shutdown
                if (!follower.isBusy()) {
                    follower.followPath(returnTo2ndScore);
                    setPathState(6);
                }
                break;
            case 6: // Return & Shutdown
                if (!follower.isBusy()) {
                    //add complete to telemetry
                    telemetry.addData("Status", "Auto Complete");
                    telemetry.update();

                }
                break;

        }
    }

    private void fireThreeTimes() {
        sleep(1000);
        fireOne();
        frontIntake.setPower(0.75);
        sleep(1200);
        frontIntake.setPower(0);
        fireOne();
        backIntake.setPower(-0.75);
        sleep(1200);
        backIntake.setPower(0);
        fireOne();
        leftFly.setPower(0.0);
        rightFly.setPower(0.0);
    }
    public void fireOne() {
        pusher.setPosition(TRIGGER_FIRE);
        sleep(800);
        pusher.setPosition(TRIGGER_HOME);
        sleep(800);
    }

    private void initMechanisms() {
        leftFly = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        pusher = hardwareMap.get(Servo.class, "trigger");
        adjuster = hardwareMap.get(Servo.class, "adjuster");
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    private void drawTelemetry() {
        // String[] debug = follower.debug();
        String[] debug = null;
        if(debug !=null) {
            telemetry.addData("Localizer", debug[0]);
            telemetry.addData("Errors", debug[1]);
            telemetry.addData("Vectors", debug[2]);
            telemetry.addData("Swerve Modules", debug[3]);
        }

        telemetry.addData("Path State", pathState);
        telemetry.update();
    }
}