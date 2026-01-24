package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
// IMU is no longer strictly needed for drive, but kept for initialization if mechanisms use it.
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//@Disabled
@TeleOp(name = "PotatoSwerve", group = "Swerve")
public class PotatoSwerve extends LinearOpMode {

    // HARDWARE DECLARATIONS
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu; // Kept IMU object, but drive logic ignores its data
    private DcMotor leftFly, rightFly;
    private DcMotor frontIntake, backIntake;
    private Servo turretRotation1, turretRotation2;

    // --- SWERVE VARIABLES --- //
    // ROBOT GEOMETRY
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // CRITICAL OFFSETS (Using your measured values)
    final double FRONT_LEFT_OFFSET  = 1.34;
    final double FRONT_RIGHT_OFFSET = 3.161;
    final double BACK_LEFT_OFFSET   = 1.589;
    final double BACK_RIGHT_OFFSET  = 1.237;

    // SWERVE TUNING PARAMETERS
    final double STEER_KP = 0.6;
    final double DRIVE_DEADBAND = 0.05;
    final double STEER_DEADBAND = 0.05;
    final double ADJUSTER_DEADBAND = 0.05; // Deadband for G2 stick control

    // --- OTHER VARIABLES --- //
    final double TRIGGER_THRESHOLD = 0.5;

    // --- ACCESSORY VARIABLES --- //
    // Speed modes
    final double MAX_SPEED_GLOBAL = 0.8;
    final double MAX_SPEED_SLOW_MODE = 0.2;


    // Wheel 'planting'
    final int FRAMES_TO_PLANT_WHEELS = 5;
    private int framesSinceLastMoved = 0;

    // Turret
    final double MIN_TURRET_ROTATION = 0.0;
    final double MAX_TURRET_ROTATION = 1.0;
    final double TURRET_ROTATION_STEP = 0.01;
    private double currentTurretRotation = (MIN_TURRET_ROTATION + MAX_TURRET_ROTATION)/2.0;

    // Flywheel
    final double FLYWHEEL_DEFAULT_POWER = 0.75;
    final double FLYWHEEL_POWER_STEP = 0.005;
    private double flyPower = FLYWHEEL_DEFAULT_POWER;
    private boolean isFlywheelOn = false;
    private boolean leftTriggerPreviouslyPressed = false;

    // Intake
    private boolean isIntakeOn = false;
    private boolean rightTriggerPreviouslyPressed = false;
    private boolean isSpinningForward = false;
    private boolean leftDpadPreviouslyPressed = false;


    @Override
    public void runOpMode() {

        initializeHardware();

        waitForStart();

        // No headingOffset is needed for Robot-Centric drive
        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;


        while (opModeIsActive()) {
            // Speed Limiter Logic (Gamepad 1)
            double speedMultiplier = MAX_SPEED_GLOBAL;
            if (gamepad1.right_bumper) {
                speedMultiplier = MAX_SPEED_SLOW_MODE;
            }

            // ------ DRIVE INPUTS (ROBOT-CENTRIC) ------ //
            // Joystick inputs are the final robot-centric inputs
            double robotY = -gamepad1.left_stick_y * speedMultiplier; // Forward/Backward
            double robotX = gamepad1.left_stick_x * speedMultiplier;  // Strafe Left/Right
            double rot = gamepad1.right_stick_x * speedMultiplier;     // Rotation

            // The coordinate transformation (IMU math) is REMOVED

            // Swerve Kinematics (Unchanged)
            double A = robotX - rot * (WHEELBASE / R);
            double B = robotX + rot * (WHEELBASE / R);
            double C = robotY - rot * (TRACK_WIDTH / R);
            double D = robotY + rot * (TRACK_WIDTH / R);

            // Calculate wheel speeds and normalize
            double speedFrontLeft  = Math.hypot(B, D);
            double speedFrontRight = Math.hypot(B, C);
            double speedBackLeft   = Math.hypot(A, D);
            double speedBackRight  = Math.hypot(A, C);
            double maxSpeed = Math.max(Math.max(speedFrontLeft, speedFrontRight), Math.max(speedBackLeft, speedBackRight));
            if (maxSpeed > 1.0) {
                speedFrontLeft /= maxSpeed;
                speedFrontRight /= maxSpeed;
                speedBackLeft /= maxSpeed;
                speedBackRight /= maxSpeed;
            }

            // Steering Logic (Unchanged)
            if (Math.abs(robotX) > DRIVE_DEADBAND || Math.abs(robotY) > DRIVE_DEADBAND || Math.abs(rot) > DRIVE_DEADBAND) {
                targetAngleFL = Math.atan2(B, D);
                targetAngleFR = Math.atan2(B, C);
                targetAngleBL = Math.atan2(A, D);
                targetAngleBR = Math.atan2(A, C);
                framesSinceLastMoved = 0;
            } else {
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
                framesSinceLastMoved += 1;
            }

            // Lock wheels override ('X' formation)
            if (gamepad1.left_stick_button || framesSinceLastMoved >= FRAMES_TO_PLANT_WHEELS) {
                targetAngleFL = -Math.PI / 4; targetAngleFR = Math.PI / 4;
                targetAngleBL = Math.PI / 4; targetAngleBR = -Math.PI / 4;
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
            }

            // Apply swerve module outputs
            runModule(frontLeftDrive, frontLeftSteer, frontLeftEncoder, FRONT_LEFT_OFFSET, speedFrontLeft, targetAngleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR);
            runModule(backLeftDrive, backLeftSteer, backLeftEncoder, BACK_LEFT_OFFSET, speedBackLeft, targetAngleBL);
            runModule(backRightDrive, backRightSteer, backRightEncoder, BACK_RIGHT_OFFSET, speedBackRight, targetAngleBR);

            // --- ACCESSORIES --- //

            // Turret
            double turretRotate = -gamepad1.right_stick_x;
            currentTurretRotation += turretRotate * TURRET_ROTATION_STEP;
            currentTurretRotation = Math.max(MIN_TURRET_ROTATION, Math.max(currentTurretRotation, MAX_TURRET_ROTATION));//just clamping

            turretRotation1.setPosition(currentTurretRotation);
            turretRotation2.setPosition(1.0 - currentTurretRotation);

            // Flywheel toggle
            boolean leftTriggerCurrentlyPressed = gamepad1.left_bumper;
            if (leftTriggerCurrentlyPressed && !leftTriggerPreviouslyPressed) {
                isFlywheelOn = !isFlywheelOn;
            }
            leftTriggerPreviouslyPressed = leftTriggerCurrentlyPressed;

            boolean flyPowerUpDpad = gamepad2.dpad_up;
            boolean flyPowerDownDpad = gamepad2.dpad_down;

            if (flyPowerUpDpad) {
                flyPower += FLYWHEEL_POWER_STEP;
            } else if (flyPowerDownDpad) {
                flyPower -= FLYWHEEL_POWER_STEP;
            }

            if (flyPower > 1.0) {
                flyPower = 1.0;
            } else if (flyPower < 0.0) {
                flyPower = 0.0;
            }

            boolean resetFlywheelPower = gamepad2.b;
            if (resetFlywheelPower) {
                flyPower = FLYWHEEL_DEFAULT_POWER;
            }

            if (isFlywheelOn) {
                leftFly.setPower(flyPower);
                rightFly.setPower(flyPower);
            }

            // Intake toggle
            boolean rightTriggerCurrentlyPressed = gamepad1.right_bumper;
            if (rightTriggerCurrentlyPressed && !rightTriggerPreviouslyPressed) {
                isIntakeOn = !isIntakeOn;
            }
            rightTriggerPreviouslyPressed = rightTriggerCurrentlyPressed;

            boolean leftDpadCurrentlyPressed = gamepad1.dpad_left;
            if (leftDpadCurrentlyPressed && !leftDpadPreviouslyPressed) {
                isSpinningForward = !isSpinningForward;
            }
            leftDpadPreviouslyPressed = leftDpadCurrentlyPressed;

            int intakeDirection = -1;
            if (isSpinningForward) {
                intakeDirection = 1;
            }

            if (isIntakeOn) {
                frontIntake.setPower(flyPower * intakeDirection);
                backIntake.setPower(flyPower * intakeDirection);
            }
        }
    }

    // --- HELPER METHODS ---

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

        leftFly = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        turretRotation1 = hardwareMap.get(Servo.class, "turret_rotation_1" );
        turretRotation2 = hardwareMap.get(Servo.class, "turret_rotation_2" );

        // --- DRIVE SETUP --- //
        // CRITICAL: IMU INITIALIZATION IS STILL NEEDED FOR HUB TO START
        // The values here don't matter for the drive function since we ignore the heading.
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
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);


        // Set Zero Power Behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset RunMode for all DC motors
        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        // --- ACCESSORY SETUP --- //
        // Turret direction
        turretRotation1.setDirection(Servo.Direction.FORWARD);
        turretRotation2.setDirection(Servo.Direction.REVERSE);

        // Flywheel direction
        frontIntake.setDirection(DcMotor.Direction.FORWARD);
        backIntake.setDirection(DcMotor.Direction.REVERSE);
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