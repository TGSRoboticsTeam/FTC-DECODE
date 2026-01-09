package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@TeleOp(name = "ComprehendingSwerve", group = "Swerve")
public class ComprehendingSwerve extends LinearOpMode {

    // --- 1. HARDWARE DECLARATIONS ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive; // Declare DCMotors
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer; // Declare CRServos
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder; // Declare Encoders
    private IMU imu; // Declare IMU
    private DcMotor leftFly, rightFly, intake; // Declare intake/outtake DCMotors
    private Servo trigger; // Declare flip-flap Servo
    private Servo adjuster; // Declare aim-curve Servo
    private Servo light; // Servo that continuously sweeps up and down, forever.

    // --- ROBOT GEOMETRY: Ohhhhhh, yeeeeah. It's caliper time. These are based on the robot model. --- //
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // --- OFFSETS: Oh, yeah! Even more caliper time! These are also based on the robot model. --- //
    final double FRONT_LEFT_OFFSET  = 5.2417;
    final double FRONT_RIGHT_OFFSET = 5.7881;
    final double BACK_LEFT_OFFSET   = 2.4143;
    final double BACK_RIGHT_OFFSET  = 4.8209;

    // --- TUNING PARAMETERS --- //
    final double STEER_LERP_WEIGHT = 0.6;
    final double DRIVE_DEADBAND = 0.05;
    final double STEER_DEADBAND = 0.05;

    // --- 5. SPEED CONTROL CONSTANTS --- //
    final double MAX_SPEED_GLOBAL = 0.8;
    final double MAX_SPEED_SLOW_MODE = 0.2;
    final double TRIGGER_THRESHOLD = 0.5;

    // --- MECHANISM DIRECTION REVERSAL --- //
    final boolean LEFT_FLY_REVERSE = false;
    final boolean RIGHT_FLY_REVERSE = true;
    final boolean INTAKE_REVERSE = true;

    // --- SERVO PARAMETERS ---
    final double SWEEP_DOWN_POSITION = 0.33;
    final double SWEEP_UP_POSITION = 0.05;
    final long SWEEP_DELAY_MS = 250;

    // Turret Tilt Control Parameters
    final double INITIAL_TURRET_TILT = 0.5;
    final double TURRET_TILT_STEP = 0.005;

    // Turret Tilt Limits (Min/Max positions for the servo)
    final double MIN_TURRET_TILT = 0.3;
    final double MAX_TURRET_TILT = 0.7;

    // --- 8. LIGHT SWEEP PARAMETERS ---
    final double LIGHT_MIN_POS = 0.277;
    final double LIGHT_MAX_POS = 0.772;
    final double LIGHT_SWEEP_STEP = 0.001;

    // --- 9. TOGGLE STATE VARIABLES ---
    private boolean isCalibrationModeActive = false;
    private boolean rightStickButtonPreviouslyPressed = false;
    private boolean isFlywheelOn = false;
    private boolean isIntakeOn = false;
    private boolean leftTriggerPreviouslyPressed = false;
    private boolean rightTriggerPreviouslyPressed = false;
    private boolean aButtonPreviouslyPressed = false;

    // Light Sweep State Variables
    private double lightSweepPosition = LIGHT_MIN_POS;
    private boolean isLightSweepingUp = true;


    @Override
    public void runOpMode() {

        initializeHardware();

        waitForStart();

        double headingOffset = 0;
        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;
        double turretTilt = INITIAL_TURRET_TILT;

        // Initialize servos
        trigger.setPosition(SWEEP_DOWN_POSITION);
        adjuster.setPosition(turretTilt);
        light.setPosition(lightSweepPosition);

        while (opModeIsActive()) {

            // --- Toggle Logic for Calibration Mode ---
            boolean rightStickButtonCurrentlyPressed = gamepad1.right_stick_button;
            if (rightStickButtonCurrentlyPressed && !rightStickButtonPreviouslyPressed) {
                isCalibrationModeActive = !isCalibrationModeActive;
            }
            rightStickButtonPreviouslyPressed = rightStickButtonCurrentlyPressed;

            // Speed Limiter Logic
            double speedMultiplier = MAX_SPEED_GLOBAL;
            if (gamepad1.right_bumper) {
                speedMultiplier = MAX_SPEED_SLOW_MODE;
            }

            // Yaw Reset Logic (Y or START button)
            if (gamepad1.start || gamepad1.y) {
                headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            // --- CALIBRATION MODE CHECK ---
            if (isCalibrationModeActive) {
                runCalibrationMode(
                        new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive},
                        new CRServo[]{frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer}
                );
                // Kill all mechanisms
                leftFly.setPower(0);
                rightFly.setPower(0);
                intake.setPower(0);
                continue;
            }

            // --- SWEEPER SERVO TRIGGER (A Button) ---
            boolean aButtonCurrentlyPressed = gamepad1.a;
            if (aButtonCurrentlyPressed && !aButtonPreviouslyPressed) {
                trigger.setPosition(SWEEP_UP_POSITION);
                sleep(SWEEP_DELAY_MS);
                trigger.setPosition(SWEEP_DOWN_POSITION);
            }
            aButtonPreviouslyPressed = aButtonCurrentlyPressed;

            // --- TURRET TILT CONTROL (D-Pad) ---
            boolean tiltUp = gamepad1.dpad_up;
            boolean tiltDown = gamepad1.dpad_down;

            if (tiltDown && turretTilt > MIN_TURRET_TILT) {
                turretTilt -= TURRET_TILT_STEP;
            } else if (tiltUp && turretTilt < MAX_TURRET_TILT) {
                turretTilt += TURRET_TILT_STEP;
            }

            adjuster.setPosition(turretTilt);
            // --- END TURRET TILT CONTROL ---


            // --- LIGHT SERVO SWEEP ---
            if (isLightSweepingUp) {
                lightSweepPosition += LIGHT_SWEEP_STEP;
                if (lightSweepPosition >= LIGHT_MAX_POS) {
                    lightSweepPosition = LIGHT_MAX_POS;
                    isLightSweepingUp = false;
                }
            } else {
                lightSweepPosition -= LIGHT_SWEEP_STEP;
                if (lightSweepPosition <= LIGHT_MIN_POS) {
                    lightSweepPosition = LIGHT_MIN_POS;
                    isLightSweepingUp = true;
                }
            }
            light.setPosition(lightSweepPosition);
            // --- END LIGHT SERVO SWEEP ---


            // --- DRIVE INPUTS (Unchanged) ---
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;
            double y = -gamepad1.left_stick_y * speedMultiplier;
            double x = gamepad1.left_stick_x * speedMultiplier;
            double rot = gamepad1.right_stick_x * speedMultiplier;

            // This stuff makes it field centric.
            double cosA = Math.cos(heading);
            double sinA = Math.sin(heading);
            double robotX = x * cosA - y * sinA;
            double robotY = x * sinA + y * cosA;

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
            if (maxSpeed > 1.0) { // Then normalize.
                speedFrontLeft /= maxSpeed;
                speedFrontRight /= maxSpeed;
                speedBackLeft /= maxSpeed;
                speedBackRight /= maxSpeed;
            }

            // Steering Logic (Unchanged)
            if (Math.abs(x) > DRIVE_DEADBAND || Math.abs(y) > DRIVE_DEADBAND || Math.abs(rot) > DRIVE_DEADBAND) {
                targetAngleFL = Math.atan2(B, D); // The angles are global, they don't get reset each frame.
                targetAngleFR = Math.atan2(B, C);
                targetAngleBL = Math.atan2(A, D);
                targetAngleBR = Math.atan2(A, C);
            } else {
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0; // If all the values are less than the DEADBAND, don't drive.
            }

            // Lock wheels override ('X' formation)
            if (gamepad1.left_stick_button) {
                targetAngleFL = Math.PI / 4; targetAngleFR = -Math.PI / 4;
                targetAngleBL = -Math.PI / 4; targetAngleBR = Math.PI / 4;
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
            }

            // Apply swerve module outputs
            runModule(frontLeftDrive, frontRightSteer, frontLeftEncoder, FRONT_LEFT_OFFSET, speedFrontLeft, targetAngleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR);
            runModule(backLeftDrive, backLeftSteer, backLeftEncoder, BACK_LEFT_OFFSET, speedBackLeft, targetAngleBL);
            runModule(backRightDrive, backRightSteer, backRightEncoder, BACK_RIGHT_OFFSET, speedBackRight, targetAngleBR);

            // --- MECHANISM CONTROL ---

            // Flywheel Toggle (Left Trigger)
            boolean leftTriggerCurrentlyPressed = gamepad1.left_trigger > TRIGGER_THRESHOLD;
            if (leftTriggerCurrentlyPressed && !leftTriggerPreviouslyPressed) {
                isFlywheelOn = !isFlywheelOn;
            }
            leftTriggerPreviouslyPressed = leftTriggerCurrentlyPressed;

            leftFly.setPower(isFlywheelOn ? 1.0 : 0);
            rightFly.setPower(isFlywheelOn ? 1.0 : 0);

            // Intake Toggle (Right Trigger)
            boolean rightTriggerCurrentlyPressed = gamepad1.right_trigger > TRIGGER_THRESHOLD;
            if (rightTriggerCurrentlyPressed && !rightTriggerPreviouslyPressed) {
                isIntakeOn = !isIntakeOn;
            }
            rightTriggerPreviouslyPressed = rightTriggerCurrentlyPressed;

            intake.setPower(isIntakeOn ? 1.0 : 0);

            // Telemetry
            telemetry.addData("Mode", "DiamondbackDrive (Field-Centric)");
            telemetry.addData("IMU Orientation Test", "Logo: LEFT, USB: RIGHT");
            telemetry.addData("Flywheel", isFlywheelOn ? "ON (Coast Stop)" : "OFF (Coast Stop)");
            telemetry.addData("Intake", isIntakeOn ? "ON" : "OFF");
            telemetry.addData("Turret Tilt", "%.3f", turretTilt);
            telemetry.addData("Light Servo Pos", "%.3f", lightSweepPosition);
            telemetry.addData("Light Direction", isLightSweepingUp ? "UP" : "DOWN");
            telemetry.update();
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

        // --- Mechanism Hardware ---
        leftFly = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");
        intake = hardwareMap.get(DcMotor.class, "intake");
        trigger = hardwareMap.get(Servo.class, "trigger");
        adjuster = hardwareMap.get(Servo.class, "adjuster");
        light = hardwareMap.get(Servo.class, "light");

        // CRITICAL: IMU ORIENTATION SETUP
        // TESTING FIX: Based on 90-degree error.
        // Rotation changed from UP to RIGHT to compensate for the observed 90-degree error.
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT    // Test Fix for 90-degree inversion
                )
        );
        imu.initialize(parameters);

        // --- Swerve Drive Motor Direction Fix ---
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // --- Mechanism Motor Direction Fix (Controlled by toggles at the top) ---
        leftFly.setDirection(LEFT_FLY_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rightFly.setDirection(RIGHT_FLY_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setDirection(INTAKE_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

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
    }

    private void runModule(DcMotor driveMotor, CRServo steerServo, AnalogInput encoder, double encoderOffset, double speed, double targetAngle) {
        // Swerve module control logic
        double rawAngle = getRawAngle(encoder); // Convert encoder voltage to radians.
        double currentAngle = rawAngle - encoderOffset; // currentAngle is the servo's real 0.00 position.
        currentAngle = wrapAngle(currentAngle); // Makes sure that currentAngle is within a PI to -PI range so it can be properly used. (-180° to 180° range)

        double angleDifference = wrapAngle(targetAngle - currentAngle); // Find the difference between the target and current angles. Wraps it to make sure it can be used further on.

        if (Math.abs(angleDifference) > Math.PI / 2) { // If it needs to rotate more than 90°...
            angleDifference = wrapAngle(angleDifference + Math.PI); // Add 180° to the rotation. So if it was at 0° and had to go to 130°, it instead goes to -50°...
            speed *= -1; // ...and flips the motor's spin direction
        }

        double servoPower = STEER_LERP_WEIGHT * angleDifference; // Smooth out how much the servo's movements. Effectively just applying lerp weight.
        servoPower *= -1; // Invert servo power to match physical rotation...

        if (Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0; // Set the servo power to 0 if withing a certain buffer range. Helps prevent jitter.

        servoPower = Math.max(-1, Math.min(1, servoPower)); // Clamp servo power to -1 to 1.

        steerServo.setPower(servoPower); // Apply servo power.
        driveMotor.setPower(speed); // Apply motor power.
    }

    private double wrapAngle(double angle) { // Makes sure an angle is within the PI to -PI range.
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;

        return angle;
    }

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private double getRawAngle(AnalogInput encoder) { // Convert encoder voltage into radians. Then return.
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }

    private void runCalibrationMode(DcMotor[] driveMotors, CRServo[] steerServos) { // For debugging.
        for (DcMotor motor : driveMotors) { motor.setPower(0); }
        for (CRServo servo : steerServos) { servo.setPower(0); }

        telemetry.addData("Mode", "**CALIBRATION - PID DISABLED**");
        telemetry.addData("FL Raw Angle", getRawAngle(frontLeftEncoder));
        telemetry.addData("FR Raw Angle", getRawAngle(frontRightEncoder));
        telemetry.addData("BL Raw Angle", getRawAngle(backLeftEncoder));
        telemetry.addData("BR Raw Angle", getRawAngle(backRightEncoder));
        telemetry.addData("Exit", "Press Right Stick Button (R3) to EXIT.");
        telemetry.update();
    }
}