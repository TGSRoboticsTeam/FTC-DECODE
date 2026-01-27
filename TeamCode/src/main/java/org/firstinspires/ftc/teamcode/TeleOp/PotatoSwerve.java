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

    // ---------------- HARDWARE ----------------
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;

    private DcMotor leftFly, rightFly;
    private DcMotor frontIntake, backIntake;

    private Servo turretRotation1, turretRotation2;
    private Servo trigger;

    // Color sensors (config names: frontColor, centerColor, backColor)
    private NormalizedColorSensor frontColor, centerColor, backColor;

    // ---------------- CONSTANTS ----------------
    // Swerve geometry
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // Analog encoder offsets
    final double FRONT_LEFT_OFFSET  = 1.34;
    final double FRONT_RIGHT_OFFSET = 3.161;
    final double BACK_LEFT_OFFSET   = 1.589;
    final double BACK_RIGHT_OFFSET  = 1.237;

    // Swerve tuning
    final double STEER_KP = 0.6;
    final double DRIVE_DEADBAND = 0.05;
    final double STEER_DEADBAND = 0.05;

    // Drive speed
    final double MAX_SPEED_GLOBAL = 0.8;

    // Wheel lock
    final int FRAMES_TO_PLANT_WHEELS = 5;

    // Trigger servo positions (two-state)
    final double TRIGGER_POS_A = 0.35;
    final double TRIGGER_POS_B = 0.55;

    // Flywheel
    final double FLYWHEEL_DEFAULT_POWER = 0.75;
    final double FLYWHEEL_POWER_STEP = 0.005;

    // Intake scaling
    final double INTAKE_FULL = 1.0;
    final double INTAKE_SLOW = 0.30; // 30%
    final double FAST_AFTER_SLOW_STOP = 0.70; // after slow stops
    final double FAST_AFTER_CENTER = 0.55;    // after slow stops AND center has ball

    // Alpha thresholds (your latest)
    final float ALPHA_ON  = 0.85f; // ball present
    final float ALPHA_OFF = 0.65f; // clear

    // Trigger threshold for toggles
    final double TRIGGER_TOGGLE_THRESHOLD = 0.5;

    // ---------------- STATE ----------------
    private int framesSinceLastMoved = 0;

    private double currentTurretRotation = 0.5;

    // Flywheel
    private double flyPower = FLYWHEEL_DEFAULT_POWER;
    private boolean isFlywheelOn = false;
    private boolean flyTriggerPrev = false; // left trigger toggle

    // Intake
    private boolean isIntakeOn = false;
    private boolean intakeTriggerPrev = false; // right trigger toggle

    // Intake mode
    // Mode 1: forward; front=fast, back=slow
    // Mode 2: reverse; back=fast, front=slow
    private boolean intakeModeOne = true;
    private boolean modeTogglePrev = false; // dpad_left edge

    // Ball latches (sticky while ball present)
    private boolean ballFront = false;
    private boolean ballCenter = false;
    private boolean ballBack = false;

    // Trigger button debounce
    private boolean aPrev = false;
    private boolean bPrev = false;

    @Override
    public void runOpMode() {

        initializeHardware();

        telemetry.addLine("PotatoSwerve ready");
        telemetry.addLine("Ball detect: alpha ON>=0.85, clear<=0.65");
        telemetry.addLine("Fast only slows/stops AFTER slow has stopped. Fast stop also requires center ball.");
        telemetry.update();

        waitForStart();

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;

        while (opModeIsActive()) {

            // ---------------- DRIVE (robot-centric) ----------------
            double robotY = -gamepad1.left_stick_y * MAX_SPEED_GLOBAL;
            double robotX =  gamepad1.left_stick_x * MAX_SPEED_GLOBAL;
            double rot    =  gamepad1.right_stick_x * MAX_SPEED_GLOBAL;

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

            // ---------------- TURRET ----------------
            currentTurretRotation += -gamepad1.right_stick_x * 0.01;
            currentTurretRotation = clamp(currentTurretRotation, 0, 1);
            turretRotation1.setPosition(currentTurretRotation);
            turretRotation2.setPosition(1.0 - currentTurretRotation);

            // ---------------- FLYWHEEL TOGGLE on LEFT TRIGGER ----------------
            boolean flyTrigger = gamepad1.left_trigger > TRIGGER_TOGGLE_THRESHOLD;
            if (flyTrigger && !flyTriggerPrev) isFlywheelOn = !isFlywheelOn;
            flyTriggerPrev = flyTrigger;

            if (gamepad2.dpad_up) flyPower += FLYWHEEL_POWER_STEP;
            if (gamepad2.dpad_down) flyPower -= FLYWHEEL_POWER_STEP;
            flyPower = clamp(flyPower, 0, 1);
            if (gamepad2.b) flyPower = FLYWHEEL_DEFAULT_POWER;

            leftFly.setPower(isFlywheelOn ? flyPower : 0);
            rightFly.setPower(isFlywheelOn ? flyPower : 0);

            // ---------------- INTAKE TOGGLE on RIGHT TRIGGER ----------------
            boolean intakeTrigger = gamepad1.right_trigger > TRIGGER_TOGGLE_THRESHOLD;
            if (intakeTrigger && !intakeTriggerPrev) isIntakeOn = !isIntakeOn;
            intakeTriggerPrev = intakeTrigger;

            // Intake mode toggle (gamepad1 dpad_left)
            boolean ld = gamepad1.dpad_left;
            if (ld && !modeTogglePrev) intakeModeOne = !intakeModeOne;
            modeTogglePrev = ld;

            // ---------------- TRIGGER SERVO (two-state) ----------------
            boolean aBtn = gamepad1.a;
            boolean bBtn = gamepad1.b;
            if (aBtn && !aPrev) trigger.setPosition(clamp(TRIGGER_POS_A, 0.0, 1.0));
            if (bBtn && !bPrev) trigger.setPosition(clamp(TRIGGER_POS_B, 0.0, 1.0));
            aPrev = aBtn;
            bPrev = bBtn;

            // ---------------- BALL DETECTION ----------------
            boolean rawFrontPresent  = alphaPresent(frontColor);
            boolean rawCenterPresent = alphaPresent(centerColor);
            boolean rawBackPresent   = alphaPresent(backColor);

            boolean rawFrontClear  = alphaClear(frontColor);
            boolean rawCenterClear = alphaClear(centerColor);
            boolean rawBackClear   = alphaClear(backColor);

            // Latch true if present
            if (rawFrontPresent)  ballFront  = true;
            if (rawCenterPresent) ballCenter = true;
            if (rawBackPresent)   ballBack   = true;

            // Reset latches when all clear
            if (rawFrontClear && rawCenterClear && rawBackClear) {
                ballFront = ballCenter = ballBack = false;
            }

            // ---------------- INTAKE CONTROL + NEW CONDITIONS ----------------
            // Mode 1: slow=BACK, fast=FRONT
            // Mode 2: slow=FRONT, fast=BACK
            boolean slowIsBack = intakeModeOne;

            boolean ballSlowPos = slowIsBack ? ballBack : ballFront;
            boolean ballFastPos = slowIsBack ? ballFront : ballBack;

            // Base commands
            double frontCmd, backCmd;
            if (intakeModeOne) {
                // Mode 1 forward
                frontCmd = +flyPower * INTAKE_FULL; // fast
                backCmd  = +flyPower * INTAKE_SLOW; // slow
            } else {
                // Mode 2 reverse
                backCmd  = -flyPower * INTAKE_FULL; // fast
                frontCmd = -flyPower * INTAKE_SLOW; // slow
            }

            boolean slowStopped = false;

            if (isIntakeOn) {
                // (1) Slow position ball -> stop slow motor
                if (ballSlowPos) {
                    if (slowIsBack) backCmd = 0;
                    else frontCmd = 0;
                    slowStopped = true;
                }

                // IMPORTANT: fast is only allowed to slow/stop if slowStopped is true.
                if (slowStopped) {

                    // (2) After slow stops -> cap fast to 70%
                    if (slowIsBack) frontCmd = limitMagnitude(frontCmd, flyPower * FAST_AFTER_SLOW_STOP);
                    else            backCmd  = limitMagnitude(backCmd,  flyPower * FAST_AFTER_SLOW_STOP);

                    // (3) If center has ball too -> cap fast further to 55%
                    if (ballCenter) {
                        if (slowIsBack) frontCmd = limitMagnitude(frontCmd, flyPower * FAST_AFTER_CENTER);
                        else            backCmd  = limitMagnitude(backCmd,  flyPower * FAST_AFTER_CENTER);
                    }

                    // (4) Fast motor may ONLY shut off if center has a ball first
                    // So: require slowStopped AND ballCenter AND ballFastPos
                    if (ballCenter && ballFastPos) {
                        if (slowIsBack) frontCmd = 0;
                        else            backCmd  = 0;
                    }
                }

                frontIntake.setPower(frontCmd);
                backIntake.setPower(backCmd);
            } else {
                frontIntake.setPower(0);
                backIntake.setPower(0);
            }

            // ---------------- TELEMETRY ----------------
            addRgbAlphaTelemetry("FRONT", frontColor);
            addRgbAlphaTelemetry("CENTER", centerColor);
            addRgbAlphaTelemetry("BACK", backColor);

            telemetry.addData("BallLatch", "F:%s C:%s B:%s",
                    ballFront ? "Y" : "N",
                    ballCenter ? "Y" : "N",
                    ballBack ? "Y" : "N");

            telemetry.addData("Intake", "%s | %s",
                    isIntakeOn ? "ON" : "OFF",
                    intakeModeOne ? "MODE1 (FWD: FrontFast BackSlow)" : "MODE2 (REV: BackFast FrontSlow)");

            telemetry.addData("Gates", "slowStopped=%s center=%s fastBall=%s",
                    slowStopped ? "Y" : "N",
                    ballCenter ? "Y" : "N",
                    ballFastPos ? "Y" : "N");

            telemetry.addData("TriggerPos", "%.3f", trigger.getPosition());

            telemetry.update();
        }
    }

    // ---------------- HELPERS ----------------

    private void initializeHardware() {
        // Swerve drive
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

        // Mechanisms
        leftFly = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");

        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake  = hardwareMap.get(DcMotor.class, "backIntake");

        turretRotation1 = hardwareMap.get(Servo.class, "turret_rotation_1");
        turretRotation2 = hardwareMap.get(Servo.class, "turret_rotation_2");

        trigger = hardwareMap.get(Servo.class, "trigger");
        trigger.setPosition(clamp(TRIGGER_POS_B, 0.0, 1.0)); // start at "B"

        // Color sensors
        frontColor  = hardwareMap.get(NormalizedColorSensor.class, "frontColor");
        centerColor = hardwareMap.get(NormalizedColorSensor.class, "centerColor");
        backColor   = hardwareMap.get(NormalizedColorSensor.class, "backColor");

        // IMU init (kept for hub startup)
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        // Drive directions (your existing)
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Flywheel directions (your update)
        leftFly.setDirection(DcMotor.Direction.REVERSE);
        rightFly.setDirection(DcMotor.Direction.FORWARD);

        // (optional but good)
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
    }

    // Present if alpha >= ALPHA_ON
    private boolean alphaPresent(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        return c.alpha >= ALPHA_ON;
    }

    // Clear if alpha <= ALPHA_OFF
    private boolean alphaClear(NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        return c.alpha <= ALPHA_OFF;
    }

    private void addRgbAlphaTelemetry(String label, NormalizedColorSensor sensor) {
        NormalizedRGBA c = sensor.getNormalizedColors();
        telemetry.addData(label, "RGB %.2f %.2f %.2f | a=%.2f", c.red, c.green, c.blue, c.alpha);
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
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double limitMagnitude(double cmd, double maxMag) {
        return Math.signum(cmd) * Math.min(Math.abs(cmd), Math.abs(maxMag));
    }
}
