package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// Import your custom constants factory


@Autonomous(name = "Pedro Swerve: Far Goal Tuned", group = "Swerve")
public class FarGoalTunedPedro extends LinearOpMode {

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
    private Pose p0,p1,p2,p3,p4,p5,p6;
    private Path path1,path2,path3,path4,path5,path6;
    public static double TARGET_TILE_INCHES = 24.0;



    public void buildPaths() {
        p0 = new Pose(9, 0, 0);
        p1 = new Pose(TARGET_TILE_INCHES*1.5, 0, 0);//In front of row 1
        p2 = new Pose(TARGET_TILE_INCHES*1.5,30 , 0); //Through row 1
        p3 = new Pose(TARGET_TILE_INCHES*2.5, 0, 0); //Front row 2
        p4 = new Pose(TARGET_TILE_INCHES*2.5, 30, 0);   //Through row 2
        p5 = new Pose(TARGET_TILE_INCHES*3.5, 0, 0); //Front row 3
        p6 = new Pose(TARGET_TILE_INCHES*3.5, 30, 0);//Through row 3

        path1 = new Path(new BezierLine(p0, p1));  //start pose to 1st row
        path1.setConstantHeadingInterpolation(0);
        path1.setTimeoutConstraint(2000);

        path2 = new Path(new BezierLine(p1, p2));//1st row to back row 1
        path2.setConstantHeadingInterpolation(0);
        path2.setTimeoutConstraint(2000);


        path3 = new Path(new BezierLine(p2, p1));//backrow 1 to front row 1
        path3.setConstantHeadingInterpolation(0);
        path3.setTimeoutConstraint(2000);


        path4 = new Path(new BezierLine(p1, p0));//backrow 1 to front row 1
        path4.setConstantHeadingInterpolation(0);
        path4.setTimeoutConstraint(2000);





    }

    @Override
    public void runOpMode() {
        // Utilize your factory method to create the Follower
        // This links your SwerveDrivetrain and SwerveLocalizer automatically
        follower = SwerveConstants.createFollower(hardwareMap);


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
            //drawTelemetry();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                follower.followPath(path1);
                setPathState(1); // Move to the state that calls follower.followPath()

                break;

            case 1:
                //Waiting to complete path 1

                if (!follower.isBusy()) {
                    //path 1 complete, action:
                    //frontIntake.setPower(1.0);
                    follower.followPath(path2);
                    setPathState(2);
                }
                break;

            case 2:
                //Waiting for path 2:row1 to end
                if (!follower.isBusy()) {
                    //End of path 2, action:
                    //frontIntake.setPower(0);
                    follower.followPath(path3);
                    setPathState(3);
                }
                break;

            case 3: // Return & Shutdown
                if (!follower.isBusy()) {
                    //fireThreeTimes();
                    follower.followPath(path4);
                    setPathState(4);
                }
                break;

            case 4: // Return & Shutdown
                if (!follower.isBusy()) {
                    follower.followPath(path5);
                    setPathState(5);
                }
                break;

            case 5: // Return & Shutdown
                if (!follower.isBusy()) {
                    follower.followPath(path6);
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