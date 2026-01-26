package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "PotatoSwerve", group = "Swerve")
public class PotatoSwerve extends LinearOpMode {

    // HARDWARE DECLARATIONS
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;

    private DcMotor leftFly, rightFly;
    private DcMotor frontIntake, backIntake;
    private Servo turretRotation1, turretRotation2;

    // REV Color Sensor V3s (config names: frontColor, centerColor, backColor)
    private NormalizedColorSensor frontColor, centerColor, backColor;

    // --- SWERVE VARIABLES ---
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

    final double MAX_SPEED_GLOBAL = 0.8;
    final double MAX_SPEED_SLOW_MODE = 0.2;

    final int FRAMES_TO_PLANT_WHEELS = 5;
    private int framesSinceLastMoved = 0;

    // Turret
    final double MIN_TURRET_ROTATION = 0.0;
    final double MAX_TURRET_ROTATION = 1.0;
    final double TURRET_ROTATION_STEP = 0.01;
    private double currentTurretRotation = 0.5;

    // Flywheel
    final double FLYWHEEL_DEFAULT_POWER = 0.75;
    final double FLYWHEEL_POWER_STEP = 0.005;
    private double flyPower = FLYWHEEL_DEFAULT_POWER;
    private boolean isFlywheelOn = false;
    private boolean leftTriggerPreviouslyPressed = false;

    // Intake
    private boolean isIntakeOn = false;
    private boolean rightTriggerPreviouslyPressed = false;

    // Two intake modes toggled by DPAD LEFT
    // Mode 1: both forward; front=100%, back=25%
    // Mode 2: both reverse; back=100%, front=25%
    private boolean intakeModeOne = true;
    private boolean leftDpadPreviouslyPressed = false;

    // Intake scaling
    final double INTAKE_FULL = 1.0;
    final double INTAKE_QUARTER = 0.25;

    @Override
    public void runOpMode() {

        initializeHardware();

        telemetry.addLine("PotatoSwerve ready");
        telemetry.addLine("Color sensors display RGB only");
        telemetry.addLine("Intake: RB toggles ON/OFF; DPAD LEFT toggles Mode 1/2");
        telemetry.update();

        waitForStart();

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;

        while (opModeIsActive()) {

            // Speed Limiter
            double speedMultiplier = gamepad1.right_bumper ? MAX_SPEED_SLOW_MODE : MAX_SPEED_GLOBAL;

            // ------ DRIVE INPUTS (ROBOT-CENTRIC) ------ //
            double robotY = -gamepad1.left_stick_y * speedMultiplier;
            double robotX =  gamepad1.left_stick_x * speedMultiplier;
            double rot    =  gamepad1.right_stick_x * speedMultiplier;

            // Swerve Kinematics
            double A = robotX - rot * (WHEELBASE / R);
            double B = robotX + rot * (WHEELBASE / R);
            double C = robotY - rot * (TRACK_WIDTH / R);
            double D = robotY + rot * (TRACK_WIDTH / R);

            double speedFL = Math.hypot(B, D);
            double speedFR = Math.hypot(B, C);
            double speedBL = Math.hypot(A, D);
            double speedBR = Math.hypot(A, C);

            double max = Math.max(Math.max(speedFL, speedFR), Math.max(speedBL, speedBR));
            if (max > 1.0) {
                speedFL /= max; speedFR /= max; speedBL /= max; speedBR /= max;
            }

            if (Math.abs(robotX) > DRIVE_DEADBAND || Math.abs(robotY) > DRIVE_DEADBAND || Math.abs(rot) > DRIVE_DEADBAND) {
                targetAngleFL = Math.atan2(B, D);
                targetAngleFR = Math.atan2(B, C);
                targetAngleBL = Math.atan2(A, D);
                targetAngleBR = Math.atan2(A, C);
                framesSinceLastMoved = 0;
            } else {
                speedFL = speedFR = speedBL = speedBR = 0;
                framesSinceLastMoved++;
            }

            // Lock wheels ('X' formation)
            if (gamepad1.left_stick_button || framesSinceLastMoved >= FRAMES_TO_PLANT_WHEELS) {
                targetAngleFL = -Math.PI / 4; targetAngleFR =  Math.PI / 4;
                targetAngleBL =  Math.PI / 4; targetAngleBR = -Math.PI / 4;
                speedFL = speedFR = speedBL = speedBR = 0;
            }

            runModule(frontLeftDrive, frontLeftSteer, frontLeftEncoder, FRONT_LEFT_OFFSET, speedFL, targetAngleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFR, targetAngleFR);
            runModule(backLeftDrive, backLeftSteer, backLeftEncoder, BACK_LEFT_OFFSET, speedBL, targetAngleBL);
            runModule(backRightDrive, backRightSteer, backRightEncoder, BACK_RIGHT_OFFSET, speedBR, targetAngleBR);

            // Turret (kept as your modifications)
            currentTurretRotation += -gamepad1.right_stick_x * TURRET_ROTATION_STEP;
            currentTurretRotation = clamp(currentTurretRotation, MIN_TURRET_ROTATION, MAX_TURRET_ROTATION);
            turretRotation1.setPosition(currentTurretRotation);
            turretRotation2.setPosition(1.0 - currentTurretRotation);

            // Flywheel toggle (gamepad1 left_bumper)
            boolean lb = gamepad1.left_bumper;
            if (lb && !leftTriggerPreviouslyPressed) isFlywheelOn = !isFlywheelOn;
            leftTriggerPreviouslyPressed = lb;

            if (gamepad2.dpad_up) flyPower += FLYWHEEL_POWER_STEP;
            if (gamepad2.dpad_down) flyPower -= FLYWHEEL_POWER_STEP;
            flyPower = clamp(flyPower, 0, 1);
            if (gamepad2.b) flyPower = FLYWHEEL_DEFAULT_POWER;

            leftFly.setPower(isFlywheelOn ? flyPower : 0);
            rightFly.setPower(isFlywheelOn ? flyPower : 0);

            // Intake toggle ON/OFF (gamepad1 right_bumper)
            boolean rb = gamepad1.right_bumper;
            if (rb && !rightTriggerPreviouslyPressed) isIntakeOn = !isIntakeOn;
            rightTriggerPreviouslyPressed = rb;

            // Intake mode toggle (gamepad1 dpad_left)
            boolean ld = gamepad1.dpad_left;
            if (ld && !leftDpadPreviouslyPressed) intakeModeOne = !intakeModeOne;
            leftDpadPreviouslyPressed = ld;

            // Apply intake behavior
            if (isIntakeOn) {
                if (intakeModeOne) {
                    // Mode 1: both forward; front=100%, back=25%
                    frontIntake.setPower(flyPower * INTAKE_FULL);
                    backIntake.setPower(flyPower * INTAKE_QUARTER);
                } else {
                    // Mode 2: both reverse; back=100%, front=25%
                    backIntake.setPower(-flyPower * INTAKE_FULL);
                    frontIntake.setPower(-flyPower * INTAKE_QUARTER);
                }
            } else {
                frontIntake.setPower(0);
                backIntake.setPower(0);
            }

            // Color telemetry (RGB only)
            addRgbTelemetry("FRONT", frontColor);
            addRgbTelemetry("CENTER", centerColor);
            addRgbTelemetry("BACK", backColor);

            telemetry.addData("Intake", "%s | %s",
                    isIntakeOn ? "ON" : "OFF",
                    intakeModeOne ? "MODE 1 (FWD: Front100 Back25)" : "MODE 2 (REV: Back100 Front25)");

            telemetry.update();
        }
    }

    // ---------------- HELPERS ----------------

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

        imu = hardwareMap.get(IMU.class, "imu");

        leftFly = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");

        turretRotation1 = hardwareMap.get(Servo.class, "turret_rotation_1");
        turretRotation2 = hardwareMap.get(Servo.class, "turret_rotation_2");

        frontColor  = hardwareMap.get(NormalizedColorSensor.class, "frontColor");
        centerColor = hardwareMap.get(NormalizedColorSensor.class, "centerColor");
        backColor   = hardwareMap.get(NormalizedColorSensor.class, "backColor");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        // Drive directions (as you had them)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Flywheel directions (your update)
        leftFly.setDirection(DcMotor.Direction.REVERSE);
        rightFly.setDirection(DcMotor.Direction.FORWARD);

        // Your other accessory config changes
        frontIntake.setDirection(DcMotor.Direction.FORWARD);
        backIntake.setDirection(DcMotor.Direction.FORWARD);

        leftFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
    }

    private void addRgbTelemetry(String label, NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        telemetry.addData(label + " RGB", "%.2f  %.2f  %.2f", c.red, c.green, c.blue);
    }

    private void runModule(DcMotor drive, CRServo steer, AnalogInput enc,
                           double offset, double speed, double target) {

        double current = wrapAngle(getRawAngle(enc) - offset);
        double delta = wrapAngle(target - current);

        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        double steerPower = clamp(-STEER_KP * delta, -1, 1);
        if (Math.abs(steerPower) < STEER_DEADBAND) steerPower = 0;

        steer.setPower(steerPower);
        drive.setPower(speed);
    }

    private double getRawAngle(AnalogInput enc) {
        return enc.getVoltage() / 3.3 * (2 * Math.PI);
    }

    private double wrapAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
