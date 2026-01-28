package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "SwerveTurretAutoAimTest", group = "Tools")
public class SwerveTurretAutoAimTest extends LinearOpMode {

    // ===================== SWERVE HARDWARE =====================
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;

    // ===================== TURRET =====================
    private Servo turretRotation1, turretRotation2;

    // Turret Rotation (your style)
    final double MIN_TURRET_ROTATION = 0.0;
    final double MAX_TURRET_ROTATION = 1.0;
    final double TURRET_ROTATION_STEP = 0.01;
    final double TURRET_DEADBAND = 0.05;
    private double currentTurretRotation = 0.5; // center points to side

    // ===================== SWERVE CONSTANTS =====================
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    final double FRONT_LEFT_OFFSET  = 1.34;
    final double FRONT_RIGHT_OFFSET = 3.161;
    final double BACK_LEFT_OFFSET   = 1.589;
    final double BACK_RIGHT_OFFSET  = 1.237;

    final double STEER_KP = 0.6;
    final double DRIVE_DEADBAND = 0.05;
    final double STEER_DEADBAND = 0.05;
    final double MAX_SPEED = 0.8;

    // ===================== VISION (APRILTAG) =====================
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // DECODE goal tags: Blue goal=ID 20, Red goal=ID 24 :contentReference[oaicite:3]{index=3}
    private static final int BLUE_GOAL_TAG_ID = 20;
    private static final int RED_GOAL_TAG_ID  = 24;

    private enum GoalTarget { BLUE, RED }
    private GoalTarget goalTarget = GoalTarget.BLUE;

    // ===================== AUTO AIM =====================
    private boolean autoAimEnabled = false;
    private boolean autoAimTogglePrev = false;

    private boolean dpadLeftPrev = false;
    private boolean dpadRightPrev = false;

    // Your gearing:
    // 1 servo rev -> 0.7885 turret rev
    private static final double TURRET_PER_SERVO = 0.7885;
    private static final double SERVO_DEG_RANGE = 355.0;

    // ΔservoPos = ΔturretDeg / (SERVO_DEG_RANGE * TURRET_PER_SERVO)
    private static final double DEG_TO_SERVO_POS = 1.0 / (SERVO_DEG_RANGE * TURRET_PER_SERVO);

    // Controller tuning
    private static final double AIM_KP = 0.9;
    private static final double AIM_DEADBAND_DEG = 1.2;
    private static final double AIM_MAX_STEP_DEG = 6.0;

    // Flip if turret turns the wrong way when auto-aiming
    private static final double AIM_SIGN = +1.0;

    @Override
    public void runOpMode() {

        initHardware();
        initVision();

        turretRotation1.setPosition(currentTurretRotation);
        turretRotation2.setPosition(1.0 - currentTurretRotation);

        telemetry.addLine("SwerveTurretAutoAimTest ready");
        telemetry.addLine("Drive: GP1 left stick translate, GP1 right stick X rotate");
        telemetry.addLine("Turret manual: GP2 right stick X (auto-aim OFF)");
        telemetry.addLine("AutoAim toggle: GP2 left bumper");
        telemetry.addLine("Target select: GP2 dpad_left=BLUE (ID20), dpad_right=RED (ID24)");
        telemetry.update();

        waitForStart();

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;

        while (opModeIsActive()) {

            // ===================== SWERVE DRIVE (ROBOT-CENTRIC) =====================
            double robotY = -gamepad1.left_stick_y * MAX_SPEED;
            double robotX =  gamepad1.left_stick_x * MAX_SPEED;
            double rot    =  gamepad1.right_stick_x * MAX_SPEED;

            if (Math.abs(robotX) < DRIVE_DEADBAND) robotX = 0;
            if (Math.abs(robotY) < DRIVE_DEADBAND) robotY = 0;
            if (Math.abs(rot)   < DRIVE_DEADBAND) rot = 0;

            double A = robotX - rot * (WHEELBASE / R);
            double B = robotX + rot * (WHEELBASE / R);
            double C = robotY - rot * (TRACK_WIDTH / R);
            double D = robotY + rot * (TRACK_WIDTH / R);

            double speedFL = Math.hypot(B, D);
            double speedFR = Math.hypot(B, C);
            double speedBL = Math.hypot(A, D);
            double speedBR = Math.hypot(A, C);

            double max = Math.max(Math.max(speedFL, speedFR), Math.max(speedBL, speedBR));
            if (max > 1.0) { speedFL/=max; speedFR/=max; speedBL/=max; speedBR/=max; }

            if (Math.abs(robotX) > DRIVE_DEADBAND || Math.abs(robotY) > DRIVE_DEADBAND || Math.abs(rot) > DRIVE_DEADBAND) {
                targetAngleFL = Math.atan2(B, D);
                targetAngleFR = Math.atan2(B, C);
                targetAngleBL = Math.atan2(A, D);
                targetAngleBR = Math.atan2(A, C);
            } else {
                speedFL = 0; speedFR = 0; speedBL = 0; speedBR = 0;
            }

            runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  speedFL, targetAngleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFR, targetAngleFR);
            runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   speedBL, targetAngleBL);
            runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  speedBR, targetAngleBR);

            // ===================== TARGET SELECT (RED/BLUE) =====================
            boolean dpadLeft = gamepad2.dpad_left;
            boolean dpadRight = gamepad2.dpad_right;

            if (dpadLeft && !dpadLeftPrev) goalTarget = GoalTarget.BLUE;
            if (dpadRight && !dpadRightPrev) goalTarget = GoalTarget.RED;

            dpadLeftPrev = dpadLeft;
            dpadRightPrev = dpadRight;

            int desiredId = (goalTarget == GoalTarget.BLUE) ? BLUE_GOAL_TAG_ID : RED_GOAL_TAG_ID;

            // ===================== AUTO AIM TOGGLE =====================
            boolean toggle = gamepad2.left_bumper;
            if (toggle && !autoAimTogglePrev) autoAimEnabled = !autoAimEnabled;
            autoAimTogglePrev = toggle;

            // ===================== TURRET CONTROL =====================
            AprilTagDetection targetDet = pickTagById(aprilTag.getDetections(), desiredId);

            if (!autoAimEnabled) {
                // Manual turret
                double turretInput = -gamepad2.right_stick_x;
                if (Math.abs(turretInput) < TURRET_DEADBAND) turretInput = 0;

                currentTurretRotation += turretInput * TURRET_ROTATION_STEP;
                currentTurretRotation = clamp(currentTurretRotation, MIN_TURRET_ROTATION, MAX_TURRET_ROTATION);

            } else {
                // Auto aim: use bearing (left/right) to center the tag :contentReference[oaicite:4]{index=4}
                if (targetDet != null && targetDet.ftcPose != null) {
                    double bearingDeg = targetDet.ftcPose.bearing;

                    if (Math.abs(bearingDeg) > AIM_DEADBAND_DEG) {
                        double cmdTurretDeg = AIM_KP * bearingDeg;
                        cmdTurretDeg = clamp(cmdTurretDeg, -AIM_MAX_STEP_DEG, AIM_MAX_STEP_DEG);

                        double deltaPos = AIM_SIGN * (cmdTurretDeg * DEG_TO_SERVO_POS);

                        currentTurretRotation = clamp(
                                currentTurretRotation + deltaPos,
                                MIN_TURRET_ROTATION,
                                MAX_TURRET_ROTATION
                        );
                    }
                }
            }

            turretRotation1.setPosition(currentTurretRotation);
            turretRotation2.setPosition(1.0 - currentTurretRotation);

            // ===================== TELEMETRY =====================
            telemetry.addData("AutoAim", autoAimEnabled ? "ON" : "OFF");
            telemetry.addData("GoalTarget", "%s (Tag ID %d)", goalTarget, desiredId);
            telemetry.addData("TurretPos", "%.3f", currentTurretRotation);

            if (targetDet != null && targetDet.ftcPose != null) {
                // Range = direct distance camera -> tag center :contentReference[oaicite:5]{index=5}
                telemetry.addData("Bearing (deg)", "%.1f", targetDet.ftcPose.bearing);
                telemetry.addData("Range (to goal)", "%.2f", targetDet.ftcPose.range);
                telemetry.addData("Yaw (deg)", "%.1f", targetDet.ftcPose.yaw);
                telemetry.addData("Elevation (deg)", "%.1f", targetDet.ftcPose.elevation);
            } else {
                telemetry.addLine("Selected goal tag not visible");
            }

            telemetry.update();
        }

        if (visionPortal != null) visionPortal.close();
    }

    private void initVision() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );
    }

    private AprilTagDetection pickTagById(List<AprilTagDetection> dets, int id) {
        if (dets == null) return null;
        for (AprilTagDetection d : dets) {
            if (d != null && d.id == id) return d;
        }
        return null;
    }

    // ===================== SWERVE HELPERS =====================
    private void runModule(DcMotor driveMotor, CRServo steerServo, AnalogInput encoder,
                           double encoderOffset, double speed, double targetAngle) {

        double rawAngle = getRawAngle(encoder);
        double currentAngle = wrapAngle(rawAngle - encoderOffset);
        double delta = wrapAngle(targetAngle - currentAngle);

        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        double servoPower = -STEER_KP * delta;
        if (Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0;
        servoPower = clamp(servoPower, -1, 1);

        steerServo.setPower(servoPower);
        driveMotor.setPower(speed);
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double getRawAngle(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void initHardware() {

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

        turretRotation1 = hardwareMap.get(Servo.class, "turret_rotation_1");
        turretRotation2 = hardwareMap.get(Servo.class, "turret_rotation_2");
        turretRotation1.setDirection(Servo.Direction.FORWARD);
        turretRotation2.setDirection(Servo.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
    }
}
