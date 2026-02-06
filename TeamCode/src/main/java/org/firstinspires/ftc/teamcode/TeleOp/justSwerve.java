package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.dashboard.canvas.Canvas;
//import com.pedropathing.localization.PoseUpdater;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.localization.PoseTracker;


@TeleOp(name = "justSwerveFieldDash", group = "Swerve")
public class justSwerve extends LinearOpMode {

    // --- 1. HARDWARE & DASHBOARD DECLARATIONS ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;
    private VoltageSensor voltageSensor;
    private HardwareMap hardwareMap;

    // PedroPathing and Dashboard objects
    //private Follower follower;
    private PoseTracker poseTracker;
    private FtcDashboard dashboard;

    // --- 2. ROBOT GEOMETRY ---
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // --- 3. OFFSETS ---
    final double FRONT_LEFT_OFFSET  = 1.34;
    final double FRONT_RIGHT_OFFSET = 3.161;
    final double BACK_LEFT_OFFSET   = 1.589;
    final double BACK_RIGHT_OFFSET  = 1.237;

    // --- 4. TUNING PARAMETERS ---
    final double STEER_KP = 0.6;
    final double DRIVE_DEADBAND = 0.05;
    final double STEER_DEADBAND = 0.05;
    final double MAX_SPEED_GLOBAL = 0.8;
    final double MAX_SPEED_SLOW_MODE = 0.2;

    private boolean isCalibrationModeActive = false;
    private boolean rightStickButtonPreviouslyPressed = false;
    final int FRAMES_TO_PLANT_WHEELS = 5;
    private int framesSinceLastMoved = 0;

    @Override
    public void runOpMode() {

        initializeHardware();

        // Initialize PedroPathing Follower and Dashboard
        //follower = new Follower(hardwareMap);
        //follower = new Follower(null);
        //poseTracker = new PoseTracker(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        waitForStart();

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;

        while (opModeIsActive()) {
            // 1. Update PedroPathing Localization
            //follower.update();

            // 2. Prepare Dashboard Packet
            TelemetryPacket packet = new TelemetryPacket();

            // Toggle Logic for Calibration Mode
            boolean rightStickButtonCurrentlyPressed = gamepad1.right_stick_button;
            if (rightStickButtonCurrentlyPressed && !rightStickButtonPreviouslyPressed) {
                isCalibrationModeActive = !isCalibrationModeActive;
            }
            rightStickButtonPreviouslyPressed = rightStickButtonCurrentlyPressed;

            // Speed Limiter Logic
            double speedMultiplier = gamepad1.right_bumper ? MAX_SPEED_SLOW_MODE : MAX_SPEED_GLOBAL;

            // Manual Yaw Reset (Field-Centric necessity)
            if (gamepad1.options) {
                imu.resetYaw();
            }

            if (isCalibrationModeActive) {
                runCalibrationMode(
                        new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive},
                        new CRServo[]{frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer}
                );
                continue;
            }

            // --- DRIVE INPUTS (FIELD-CENTRIC) ---
            double fieldY = -gamepad1.left_stick_y * speedMultiplier;
            double fieldX = gamepad1.left_stick_x * speedMultiplier;
            double rot = gamepad1.right_stick_x * speedMultiplier;

            double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate inputs into Robot Coordinate System
            double robotX = fieldX * Math.cos(-robotYaw) - fieldY * Math.sin(-robotYaw);
            double robotY = fieldX * Math.sin(-robotYaw) + fieldY * Math.cos(-robotYaw);

            // Swerve Kinematics
            double A = robotX - rot * (WHEELBASE / R);
            double B = robotX + rot * (WHEELBASE / R);
            double C = robotY - rot * (TRACK_WIDTH / R);
            double D = robotY + rot * (TRACK_WIDTH / R);

            double speedFrontLeft  = Math.hypot(B, D);
            double speedFrontRight = Math.hypot(B, C);
            double speedBackLeft   = Math.hypot(A, D);
            double speedBackRight  = Math.hypot(A, C);

            double maxSpeed = Math.max(Math.max(speedFrontLeft, speedFrontRight), Math.max(speedBackLeft, speedBackRight));
            if (maxSpeed > 1.0) {
                speedFrontLeft /= maxSpeed; speedFrontRight /= maxSpeed;
                speedBackLeft /= maxSpeed; speedBackRight /= maxSpeed;
            }

            // Steering/Movement check
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

            // X-Lock formation
            if (gamepad1.left_stick_button || framesSinceLastMoved >= FRAMES_TO_PLANT_WHEELS) {
                targetAngleFL = -Math.PI / 4; targetAngleFR = Math.PI / 4;
                targetAngleBL = Math.PI / 4; targetAngleBR = -Math.PI / 4;
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
            }

            // Apply module outputs
            runModule(frontLeftDrive, frontLeftSteer, frontLeftEncoder, FRONT_LEFT_OFFSET, speedFrontLeft, targetAngleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR);
            runModule(backLeftDrive, backLeftSteer, backLeftEncoder, BACK_LEFT_OFFSET, speedBackLeft, targetAngleBL);
            runModule(backRightDrive, backRightSteer, backRightEncoder, BACK_RIGHT_OFFSET, speedBackRight, targetAngleBR);

            // --- DASHBOARD UPDATE ---
          //  follower.drawOnDash(packet); // Draws robot on field view
            //--------------------------DrawDash-----//
            // --- REPLACEMENT FOR drawOnDash WITHOUT POSE CLASS ---
// 1. Get primitives directly from the follower
          //  double robotFX = follower.getPose().getX();
           // double robotFY = follower.getPose().getY();
            //double robotHeading = follower.getPose().getHeading();

// 2. Access the dashboard canvas
            Canvas fieldOverlay = packet.fieldOverlay();

// 3. Draw the Robot Body (18x18 inch square equivalent)
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#3F51B5"); // Blue for the robot
           // fieldOverlay.strokeCircle(robotFX, robotFY, 9); // 9-inch radius circle

// 4. Draw the "Front" Vector (Heading)
// We calculate the end point of a line based on the robot's rotation
            double lineLength = 12.0;
           // double endX = robotFX + Math.cos(robotHeading) * lineLength;
          //  double endY = robotFY + Math.sin(robotHeading) * lineLength;

            fieldOverlay.setStroke("#FF0000"); // Red for the heading line
           // fieldOverlay.strokeLine(robotFX, robotFY, endX, endY);

            //******************************//

            packet.put("Field Heading (Deg)", Math.toDegrees(robotYaw));
           // packet.put("Robot Pose", follower.getPose().toString());

            dashboard.sendTelemetryPacket(packet);
        }
    }

    // (Helper methods initializeHardware, runModule, etc. remain the same as your original)
    private void initializeHardware() {
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
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();


        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
    }

    private void runModule(DcMotor driveMotor, CRServo steerServo, AnalogInput encoder, double encoderOffset, double speed, double targetAngle) {
        double currentAngle = wrapAngle(getRawAngle(encoder) - encoderOffset);
        double delta = wrapAngle(targetAngle - currentAngle);

        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        double servoPower = Math.max(-1, Math.min(1, STEER_KP * delta * -1));
        if (Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0;

        steerServo.setPower(servoPower);
        driveMotor.setPower(speed);
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getRawAngle(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }

    private void runCalibrationMode(DcMotor[] driveMotors, CRServo[] steerServos) {
        for (DcMotor motor : driveMotors) motor.setPower(0);
        for (CRServo servo : steerServos) servo.setPower(0);
        telemetry.update();
    }
}