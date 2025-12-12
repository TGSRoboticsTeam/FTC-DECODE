package org.firstinspires.ftc.teamcode.Autonomous; // Changed package name for organization

import com.qualcomm.robotcore.eventloop.opmode.Autonomous; // Changed OpMode type
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime; // Import for timing

// IMPORTANT: This autonomous mode is **Robot-Centric** because it does not use the IMU heading
// for field-centric corrections. The robot will move relative to its current orientation.

@Autonomous(name = "DiamondbackSimpleAuto", group = "Swerve")
public class DiamondbackDriveSimpleAuto extends LinearOpMode {

    // --- 1. HARDWARE DECLARATIONS ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu; // Kept for initialization
    private DcMotor leftFly, rightFly, intake;
    private Servo trigger;
    private Servo adjuster;
    private Servo light;
    private ElapsedTime runtime = new ElapsedTime(); // Timer for autonomous

    // --- 2. ROBOT GEOMETRY (Copied from TeleOp) ---
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // --- 3. CRITICAL: OFFSETS (Copied from TeleOp) ---
    final double FRONT_LEFT_OFFSET  = 5.2417;
    final double FRONT_RIGHT_OFFSET = 5.7881;
    final double BACK_LEFT_OFFSET   = 2.4143;
    final double BACK_RIGHT_OFFSET  = 4.8209;

    // --- 4. TUNING PARAMETERS (Copied from TeleOp) ---
    final double STEER_KP = 0.6;
    final double DRIVE_DEADBAND = 0.05;
    final double STEER_DEADBAND = 0.05;

    // --- 5. DRIVE CONSTANTS ---
    final double DRIVE_SPEED = 0.5; // Constant speed for autonomous moves

    // --- 6. SERVO/MECHANISM CONSTANTS (Minimized for simple auto) ---
    final double SWEEP_DOWN_POSITION = 0.33;
    final double INITIAL_TURRET_TILT = 0.5;

    // Swerve module target angles (persistent across drive steps)
    private double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;


    @Override
    public void runOpMode() {

        initializeHardware();

        // Ensure mechanisms are in a safe starting position
        trigger.setPosition(SWEEP_DOWN_POSITION);
        adjuster.setPosition(INITIAL_TURRET_TILT);
        // Light servo doesn't need to sweep in auto, just set a position
        light.setPosition(0.5);

        telemetry.addData("Status", "Hardware Initialized. Awaiting Start.");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // --- 1. DRIVE BACKWARD FOR 1 SECOND ---
            telemetry.addData("Auto Step", "1: Driving Backward (1 sec)");
            telemetry.update();

            // Robot-Centric Inputs: Y (Forward/Backward), X (Strafe), Rot (Rotation)
            // Backward is -Y, so set robotY to -DRIVE_SPEED
            // No strafe (robotX = 0), No rotation (rot = 0)
            driveSwerve(0, -DRIVE_SPEED, 0, DRIVE_SPEED);

            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 1.0) {
                // Keep the drive modules running for 1 second
                idle();
            }

            // --- 2. STOP AND WAIT FOR 8 SECONDS ---
            telemetry.addData("Auto Step", "2: Waiting (8 sec)");
            telemetry.update();

            // Stop the robot completely (robotY=0, robotX=0, rot=0)
            driveSwerve(0, 0, 0, 0);

            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 8.0) {
                // Keep the wheels straight/stopped while waiting
                idle();
            }

            // --- 3. DRIVE AT 45 DEGREES (Forward-Right) FOR 3 SECONDS ---
            telemetry.addData("Auto Step", "3: Driving 45 Degrees (3 sec)");
            telemetry.update();

            // Driving forward and right requires robotY > 0 and robotX > 0
            // For a 45-degree angle: robotX = robotY
            double driveX = DRIVE_SPEED * Math.sin(Math.PI / 4); // sin(45 deg)
            double driveY = DRIVE_SPEED * Math.cos(Math.PI / 4); // cos(45 deg)

            // Note: The swerve kinematics will combine these components to determine
            // the module angles and speeds for a net 45-degree move.
            driveSwerve(driveX, driveY, 0, DRIVE_SPEED);

            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < 3.0) {
                // Keep the drive modules running for 3 seconds
                idle();
            }

            // --- 4. FINAL STOP ---
            telemetry.addData("Auto Step", "4: Final Stop");
            telemetry.update();
            driveSwerve(0, 0, 0, 0);
        }
    }

    // --- NEW: Simplified Drive Function for Autonomous ---
    private void driveSwerve(double robotX, double robotY, double rot, double maxSpeed) {

        // This function recalculates the swerve kinematics and updates the modules once.

        // Swerve Kinematics (Copied from TeleOp)
        double A = robotX - rot * (WHEELBASE / R);
        double B = robotX + rot * (WHEELBASE / R);
        double C = robotY - rot * (TRACK_WIDTH / R);
        double D = robotY + rot * (TRACK_WIDTH / R);

        // Calculate wheel speeds
        double speedFrontLeft  = Math.hypot(B, D);
        double speedFrontRight = Math.hypot(B, C);
        double speedBackLeft   = Math.hypot(A, D);
        double speedBackRight  = Math.hypot(A, C);

        // Normalization (using the maxSpeed parameter for desired power level)
        double maxCalculatedSpeed = Math.max(Math.max(speedFrontLeft, speedFrontRight), Math.max(speedBackLeft, speedBackRight));

        // If there's movement, normalize and scale to maxSpeed
        if (maxCalculatedSpeed > 0.0) {
            double ratio = maxSpeed / maxCalculatedSpeed;
            speedFrontLeft  *= ratio;
            speedFrontRight *= ratio;
            speedBackLeft   *= ratio;
            speedBackRight  *= ratio;
        }

        // Steering Logic: Only recalculate angles if there is significant movement
        if (Math.abs(robotX) > DRIVE_DEADBAND || Math.abs(robotY) > DRIVE_DEADBAND || Math.abs(rot) > DRIVE_DEADBAND) {
            targetAngleFL = Math.atan2(B, D);
            targetAngleFR = Math.atan2(B, C);
            targetAngleBL = Math.atan2(A, D);
            targetAngleBR = Math.atan2(A, C);
        } else {
            // If stopped, set drive speeds to 0 but keep target angles locked for brake
            speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
            // The existing target angles will keep the wheels pointed, acting as a brake.
        }

        // Apply swerve module outputs
        runModule(frontLeftDrive, frontLeftSteer, frontLeftEncoder, FRONT_LEFT_OFFSET, speedFrontLeft, targetAngleFL);
        runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR);
        runModule(backLeftDrive, backLeftSteer, backLeftEncoder, BACK_LEFT_OFFSET, speedBackLeft, targetAngleBL);
        runModule(backRightDrive, backRightSteer, backRightEncoder, BACK_RIGHT_OFFSET, speedBackRight, targetAngleBR);
    }

    // --- HELPER METHODS (Copied from TeleOp) ---

    private void initializeHardware() {
        // --- Swerve Drive Hardware ---
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftSteer  = hardwareMap.get(CRServo.class, "frontLeftSteer");
        frontRightSteer = hardwareMap.get(CRServo.class, "frontRightSteer");
        backLeftSteer   = hardwareMap.get(CRServo.class, "backLeftSteer");
        backRightSteer  = hardwareMap.get(CRServo.class, "backRightSteer");
        frontLeftEncoder  = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder   = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder  = hardwareMap.get(AnalogInput.class, "backRightEncoder");
        imu = hardwareMap.get(IMU.class, "imu");

        // --- Mechanism Hardware ---
        leftFly = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");
        intake = hardwareMap.get(DcMotor.class, "intake");
        trigger = hardwareMap.get(Servo.class, "trigger");
        adjuster = hardwareMap.get(Servo.class, "adjuster");
        light = hardwareMap.get(Servo.class, "light");

        // CRITICAL: IMU INITIALIZATION IS STILL NEEDED FOR HUB TO START
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        // --- Swerve Drive Motor Direction Fix ---
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // --- Mechanism Motor Direction Fix (Set to 0 power in auto) ---
        leftFly.setDirection(DcMotorSimple.Direction.FORWARD); // Defaulted to FORWARD for simplicity in auto
        rightFly.setDirection(DcMotorSimple.Direction.REVERSE); // Defaulted to REVERSE for simplicity in auto
        intake.setDirection(DcMotorSimple.Direction.REVERSE); // Defaulted to REVERSE for simplicity in auto

        // Set Zero Power Behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset RunMode for all DC motors
        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, leftFly, rightFly, intake);

        // Ensure all mechanism motors are off at the start of auto
        leftFly.setPower(0);
        rightFly.setPower(0);
        intake.setPower(0);
    }

    private void runModule(DcMotor driveMotor, CRServo steerServo, AnalogInput encoder, double encoderOffset, double speed, double targetAngle) {
        // Swerve module control logic (unchanged)
        double rawAngle = getRawAngle(encoder);
        double currentAngle = rawAngle - encoderOffset;
        currentAngle = wrapAngle(currentAngle);

        double delta = wrapAngle(targetAngle - currentAngle);

        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        double servoPower = STEER_KP * delta;
        servoPower *= -1; // Steering Fix: Invert servo power to match physical rotation

        if (Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0;

        servoPower = Math.max(-1, Math.min(1, servoPower));

        steerServo.setPower(servoPower);
        driveMotor.setPower(speed);
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private double getRawAngle(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }
}