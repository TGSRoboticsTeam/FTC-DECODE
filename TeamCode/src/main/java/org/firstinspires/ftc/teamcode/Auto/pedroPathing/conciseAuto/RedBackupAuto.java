package org.firstinspires.ftc.teamcode.Auto.pedroPathing.conciseAuto;

import static org.firstinspires.ftc.teamcode.Auto.pedroPathing.conciseAuto.FieldPaths.Colorado;
import static org.firstinspires.ftc.teamcode.Auto.pedroPathing.conciseAuto.FieldPaths.NewMexico;
import static org.firstinspires.ftc.teamcode.Auto.pedroPathing.conciseAuto.FieldPose.redDepot;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Auto.pedroPathing.Constants;


@Autonomous(name = "RedBackup Auto", group = "Examples")
public class RedBackupAuto extends OpMode {

    private Follower follower;
    private int pathIndex = 0;
    private static DcMotor leftFlywheel;
    private static DcMotor rightFlywheel;
    private static Servo leftFlap;
    private static Servo rightFlap;
    private static Servo light;
    private static Timer timer;
    DcMotor leftFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightFrontDrive;
    DcMotor rightBackDrive;





    // The new sequence array, holding both paths and events
    private PathAndEvent[] pathSequence;

    @Override
    public void init() {
        // Initialize the follower and field paths
        follower = Constants.createFollower(hardwareMap);
        FieldPaths.initializePaths(follower);
        follower.setStartingPose(redDepot);
        leftFlywheel = hardwareMap.get(DcMotor.class, "left_fly");
        rightFlywheel = hardwareMap.get(DcMotor.class, "right_fly");
        leftFlap = hardwareMap.get(Servo.class, "left_flap");
        rightFlap = hardwareMap.get(Servo.class, "right_flap");
        light = hardwareMap.get(Servo.class, "light");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Sets the motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Makes the motors stop moving when they receive an input of 0
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



// Sets the motor direction
        leftFlywheel.setDirection(DcMotor.Direction.FORWARD);
        rightFlywheel.setDirection(DcMotor.Direction.REVERSE);

        // Makes the motors stop moving when they receive an input of 0
        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        // Initialize the FieldEvent class with the hardware map
        FieldEvent.initialize(hardwareMap);

        // Define the path sequence with an event after each path
/*
        pathSequence = new PathAndEvent[]{
                new PathAndEvent(redDepotShoot, Event.SHOOT),
                new PathAndEvent(redShootCenter, Event.CALIBRATE),
                new PathAndEvent(CENTER_F2, Event.NULL),
                new PathAndEvent(F2_CENTER, Event.NULL), // Pause for 2 seconds
                new PathAndEvent(F2_CENTER, Event.NULL), // No event
                new PathAndEvent(CENTER_A5, Event.CALIBRATE),
                new PathAndEvent(A5_CENTER, Event.NULL),
                new PathAndEvent(CENTER_F5, Event.READ),
                new PathAndEvent(F5_CENTER, Event.NULL) // No event
        };
*/

        pathSequence = new PathAndEvent[]{
                new PathAndEvent(Colorado, Event.SHOOT3),
                new PathAndEvent(NewMexico, Event.NULL)
        };



    }
    @Override
    public void start() {
        pauseTime(17);
        leftFlywheel.setPower(1);
        rightFlywheel.setPower(1);

        leftFrontDrive.setPower(.25);
        leftBackDrive.setPower(.25);
        rightFrontDrive.setPower(.25);
        rightBackDrive.setPower(.25);

        pauseTime(2.5);

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);


        pauseTime(0.5);

        leftFlap.setPosition(0.25);
        rightFlap.setPosition(0.75);
        pauseTime(1);
        leftFlap.setPosition(0);
        rightFlap.setPosition(1);
        pauseTime(1);
        leftFlap.setPosition(0.25);
        rightFlap.setPosition(0.75);
        pauseTime(1);
        leftFlap.setPosition(0);
        rightFlap.setPosition(1);
        pauseTime(1);
        leftFlap.setPosition(0.25);
        rightFlap.setPosition(0.75);
        pauseTime(1);
        leftFlap.setPosition(0);
        rightFlap.setPosition(1);

        pauseTime(1);
        leftFlap.setPosition(0.25);
        rightFlap.setPosition(0.75);
        pauseTime(1);
        leftFlap.setPosition(0);
        rightFlap.setPosition(1);

        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
        pauseTime(1);
        //
        leftFrontDrive.setPower(-.35);
        leftBackDrive.setPower(.35);
        rightFrontDrive.setPower(.35);
        rightBackDrive.setPower(-.35);

        pauseTime(1.5);

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        pathIndex = 0;
        // Start the first path immediately
        if (pathIndex < pathSequence.length) {
           // follower.followPath(pathSequence[pathIndex].path);
        }
    }

    @Override
    public void loop() {
        /*
        follower.update();

        // If the current path is complete, perform the event and move to the next path
        if (!follower.isBusy()) {
            if (pathIndex < pathSequence.length) {
                // Perform the event associated with the completed path
                FieldEvent.perform(pathSequence[pathIndex].event);

                // Then, move to the next path in the sequence
                pathIndex++;

                // Follow the next path if it exists
                if (pathIndex < pathSequence.length) {
                    follower.followPath(pathSequence[pathIndex].path);
                } else {
                    // All paths are complete, set index to a final state
                    pathIndex = -1;
                }
            }
        }
  */
        // Telemetry for debugging
        telemetry.addData("Path Index", pathIndex);
        telemetry.addData("Current Event", pathIndex > -1 && pathIndex < pathSequence.length ? pathSequence[pathIndex].event : "N/A");
        telemetry.update();
    }

    @Override
    public void stop() {
        // You can add logic here to stop all subsystems
    }
    public static void pauseTime(double t){
        Timer timer2 = new Timer();
        timer2.resetTimer();
        while(timer2.getElapsedTimeSeconds() < t){
        }
    }
}