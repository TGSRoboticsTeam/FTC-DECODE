package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "FieldCentricSwerving", group = "Swerve")
public class FieldCentricSwerving extends LinearOpMode {

    // --- 1. HARDWARE DECLARATIONS ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;
    private VoltageSensor voltageSensor;

    // --- 2. ROBOT GEOMETRY ---
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // --- 3. OFFSETS (Your measured values) ---
    final double FRONT_LEFT_OFFSET  = 1.34;
    final double FRONT_RIGHT_OFFSET = 3.161;
    final double BACK_LEFT_OFFSET   = 1.589;
    final double BACK_RIGHT_OFFSET  = 1.237;

    // --- 4. TUNING PARAMETERS ---
    final double STEER_KP = 0.6;
    final double DRIVE_DEADBAND = 0.05;
    final double STEER_DEADBAND = 0.05;

    // --- 5. SPEED CONTROL CONSTANTS ---
    final double MAX_SPEED_GLOBAL = 0.8;
    final double MAX_SPEED_SLOW_MODE = 0.2;

    // --- 9. TOGGLE STATE VARIABLES ---
    private boolean isCalibrationModeActive = false;
    private boolean rightStickButtonPreviouslyPressed = false;

    // Yaw reset (Gamepad1 Y)
    private boolean yPreviouslyPressed = false;

    // Wheel 'planting'
    final int FRAMES_TO_PLANT_WHEELS = 5;
    private int framesSinceLastMoved = 0;

    @Override
    public void runOpMode() {

        initializeHardware();

        waitForStart();

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;

        while (opModeIsActive()) {

            // --- Toggle Logic for Calibration Mode (Gamepad 1 R3) ---
            boolean rightStickButtonCurrentlyPressed = gamepad1.right_stick_button;
            if (rightStickButtonCurrentlyPressed && !rightStickButtonPreviouslyPressed) {
                isCalibrationModeActive = !isCalibrationModeActive;
            }
            rightStickButtonPreviouslyPressed = rightStickButtonCurrentlyPressed;

            // --- Speed Limiter Logic (Gamepad 1 RB) ---
            double speedMultiplier = MAX_SPEED_GLOBAL;
            if (gamepad1.right_bumper) {
                speedMultiplier = MAX_SPEED_SLOW_MODE;
            }

            // --- Yaw reset (Gamepad 1 Y) ---
            // Tap Y when robot is facing the direction you want to be "field forward".
            boolean yPressed = gamepad1.y;
            if (yPressed && !yPreviouslyPressed) {
                imu.resetYaw();
            }
            yPreviouslyPressed = yPressed;

            // --- CALIBRATION MODE CHECK ---
            if (isCalibrationModeActive) {
                runCalibrationMode(
                        new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive},
                        new CRServo[]{frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer}
                );
                continue;
            }

            // ----------------------------
            // FIELD-CENTRIC DRIVER INPUTS
            // ----------------------------
            // Driver frame (field frame):
            double xField = gamepad1.left_stick_x * speedMultiplier;   // strafe
            double yField = -gamepad1.left_stick_y * speedMultiplier;  // forward
            double rot    = gamepad1.right_stick_x * speedMultiplier;  // rotation (robot-centric)

            // Convert field -> robot using IMU heading
            double heading = getHeadingRadians();

            double cosA = Math.cos(-heading);
            double sinA = Math.sin(-heading);

            double robotX = xField * cosA - yField * sinA;
            double robotY = xField * sinA + yField * cosA;

            // ----------------------------
            // SWERVE KINEMATICS (unchanged)
            // ----------------------------
            double A = robotX - rot * (WHEELBASE / R);
            double B = robotX + rot * (WHEELBASE / R);
            double C = robotY - rot * (TRACK_WIDTH / R);
            double D = robotY + rot * (TRACK_WIDTH / R);

            // Calculate wheel speeds and normalize
            double speedFrontLeft  = Math.hypot(B, D);
            double speedFrontRight = Math.hypot(B, C);
            double speedBackLeft   = Math.hypot(A, D);
            double speedBackRight  = Math.hypot(A, C);

            double maxSpeed = Math.max(
                    Math.max(speedFrontLeft, speedFrontRight),
                    Math.max(speedBackLeft, speedBackRight)
            );
            if (maxSpeed > 1.0) {
                speedFrontLeft  /= maxSpeed;
                speedFrontRight /= maxSpeed;
                speedBackLeft   /= maxSpeed;
                speedBackRight  /= maxSpeed;
            }

            // Steering target angles
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
                targetAngleFL = -Math.PI / 4; targetAngleFR =  Math.PI / 4;
                targetAngleBL =  Math.PI / 4; targetAngleBR = -Math.PI / 4;
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
            }

            // Apply swerve module outputs
            runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  speedFrontLeft,  targetAngleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR);
            runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   speedBackLeft,   targetAngleBL);
            runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  speedBackRight,  targetAngleBR);

            telemetry.addData("Mode", "FIELD-CENTRIC (IMU)");
            telemetry.addData("Yaw (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Cal Mode", isCalibrationModeActive);
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

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // --- IMU ---
        imu = hardwareMap.get(IMU.class, "imu");

        // You said: USB points BACK, Logo points LEFT (robot perspective)
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        imu.initialize(parameters);

        // Start with yaw = 0 (recommended)
        imu.resetYaw();

        // --- Drive Motor Direction (kept as you had it) ---
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
    }

    private double getHeadingRadians() {
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        return ypr.getYaw(AngleUnit.RADIANS);
    }

    private void runModule(DcMotor driveMotor, CRServo steerServo, AnalogInput encoder,
                           double encoderOffset, double speed, double targetAngle) {

        double rawAngle = getRawAngle(encoder);
        double currentAngle = wrapAngle(rawAngle - encoderOffset);

        double delta = wrapAngle(targetAngle - currentAngle);

        // Optimize: flip wheel direction if > 90° away
        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        double servoPower = STEER_KP * delta;

        // Your steering invert fix
        servoPower *= -1;

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

    private void runCalibrationMode(DcMotor[] driveMotors, CRServo[] steerServos) {
        for (DcMotor motor : driveMotors) motor.setPower(0);
        for (CRServo servo : steerServos) servo.setPower(0);

        telemetry.addData("Mode", "**CALIBRATION - PID DISABLED**");
        telemetry.addData("Exit", "Press R3 to EXIT.");
        telemetry.update();
    }
}
