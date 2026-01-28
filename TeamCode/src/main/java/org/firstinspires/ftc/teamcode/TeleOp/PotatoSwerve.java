package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "PotatoSwerve", group = "Swerve")
public class PotatoSwerve extends LinearOpMode {

    /* ===================== DRIVE HARDWARE ===================== */
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;

    /* ===================== MECHANISMS ===================== */
    private DcMotor frontIntake, backIntake;
    private DcMotor leftFly, rightFly;

    private Servo turretRotation1, turretRotation2;
    private Servo trigger;

    /* ===================== COLOR SENSORS ===================== */
    private NormalizedColorSensor frontColor, centerColor, backColor;

    /* ===================== SWERVE CONSTANTS ===================== */
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

    /* ===================== TURRET (GAMEPAD2 RIGHT STICK X) ===================== */
    final double MIN_TURRET_ROTATION = 0.0;
    final double MAX_TURRET_ROTATION = 1.0;
    final double TURRET_ROTATION_STEP = 0.01;
    final double TURRET_DEADBAND = 0.05;
    private double currentTurretRotation =
            (MIN_TURRET_ROTATION + MAX_TURRET_ROTATION) / 2.0;

    /* ===================== MANUAL INTAKE TOGGLES ===================== */
    private boolean isIntakeOn = false;
    private boolean intakeTogglePrev = false;

    // Mode toggle (kept from your earlier design)
    // Mode 1: IN (front is FAST, back is SLOW)
    // Mode 2: OUT (back is FAST, front is SLOW)
    private boolean intakeModeOne = true;
    private boolean intakeModePrev = false;

    // Base powers
    final double MANUAL_FAST = 1.0;
    final double MANUAL_SLOW = 0.30;

    // Smart gating powers for FAST once slow has captured
    final double FAST_AFTER_SLOW = 0.70;
    final double FAST_AFTER_SLOW_AND_CENTER = 0.55;

    /* ===================== LAUNCH / INTAKE ===================== */
    final double LAUNCH_INTAKE_POWER = 0.76;

    /* ===================== TRIGGER SERVO (CALIBRATED) ===================== */
    // You said: 0.0 = launched state, 0.225 = down/reset
    final double TRIGGER_FIRE = 0.0;
    final double TRIGGER_HOME = 0.225;
    final long   TRIGGER_PULSE_MS = 250; // make longer if needed

    /* ===================== FLYWHEEL TIMING ===================== */
    final long FLYWHEEL_SPINUP_MS = 1000;     // no encoders
    final long FLYWHEEL_SPINDOWN_MS = 300;

    /* ===================== BALL DETECTION (ALPHA) ===================== */
    final float ALPHA_ON  = 0.85f;
    final float ALPHA_OFF = 0.65f;  // hysteresis (stable “release” threshold)

    // Latched presence (hysteresis)
    private boolean frontHasBall = false;
    private boolean centerHasBall = false;
    private boolean backHasBall = false;

    // For detecting a NEW center ball (rising edge)
    private boolean centerPrevRaw = false;

    /* ===================== LAUNCH STATE MACHINE ===================== */
    private enum LaunchState {
        IDLE,
        SPINUP,
        FIRE_FRONT,
        FEED_FRONT,
        FIRE_BACK,
        FEED_BACK,
        SPINDOWN
    }

    private LaunchState launchState = LaunchState.IDLE;
    private long stateTimer = 0;

    // If intake was ON before launching, resume after sequence
    private boolean intakeWasOnBeforeLaunch = false;

    @Override
    public void runOpMode() {

        initHardware();

        // Safe initial positions
        trigger.setPosition(TRIGGER_HOME);
        turretRotation1.setPosition(currentTurretRotation);
        turretRotation2.setPosition(1.0 - currentTurretRotation);

        waitForStart();

        while (opModeIsActive()) {

            /* ===================== DRIVE (ROBOT-CENTRIC) ===================== */
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

            double spFL = Math.hypot(B, D);
            double spFR = Math.hypot(B, C);
            double spBL = Math.hypot(A, D);
            double spBR = Math.hypot(A, C);

            double max = Math.max(Math.max(spFL, spFR), Math.max(spBL, spBR));
            if (max > 1.0) { spFL/=max; spFR/=max; spBL/=max; spBR/=max; }

            runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  spFL, Math.atan2(B, D));
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, spFR, Math.atan2(B, C));
            runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   spBL, Math.atan2(A, D));
            runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  spBR, Math.atan2(A, C));

            /* ===================== TURRET (GAMEPAD2 RIGHT STICK X) ===================== */
            double turretRotate = -gamepad2.right_stick_x;
            if (Math.abs(turretRotate) < TURRET_DEADBAND) turretRotate = 0;

            currentTurretRotation += turretRotate * TURRET_ROTATION_STEP;
            currentTurretRotation = clamp(currentTurretRotation, MIN_TURRET_ROTATION, MAX_TURRET_ROTATION);

            turretRotation1.setPosition(currentTurretRotation);
            turretRotation2.setPosition(1.0 - currentTurretRotation);

            /* ===================== READ SENSORS + UPDATE LATCHED BALL FLAGS ===================== */
            NormalizedRGBA f = frontColor.getNormalizedColors();
            NormalizedRGBA c = centerColor.getNormalizedColors();
            NormalizedRGBA b = backColor.getNormalizedColors();

            frontHasBall  = hysteresisBall(frontHasBall,  f.alpha);
            centerHasBall = hysteresisBall(centerHasBall, c.alpha);
            backHasBall   = hysteresisBall(backHasBall,   b.alpha);

            // NEW center ball edge (used by launch sequence)
            boolean centerRaw = (c.alpha >= ALPHA_ON);
            boolean centerNewBall = centerRaw && !centerPrevRaw;
            centerPrevRaw = centerRaw;

            /* ===================== MANUAL INTAKE CONTROLS (ONLY WHEN NOT LAUNCHING) ===================== */
            if (launchState == LaunchState.IDLE) {

                // Toggle intake ON/OFF with right trigger
                boolean intakeToggle = gamepad1.right_trigger > 0.5;
                if (intakeToggle && !intakeTogglePrev) {
                    isIntakeOn = !isIntakeOn;
                }
                intakeTogglePrev = intakeToggle;

                // Toggle intake mode with dpad_left
                boolean modeToggle = gamepad1.dpad_left;
                if (modeToggle && !intakeModePrev) {
                    intakeModeOne = !intakeModeOne;
                }
                intakeModePrev = modeToggle;

                if (isIntakeOn) {
                    applySmartIntake(intakeModeOne);
                } else {
                    frontIntake.setPower(0);
                    backIntake.setPower(0);
                }
            }

            /* ===================== LAUNCH SEQUENCE (A BUTTON) ===================== */
            switch (launchState) {

                case IDLE:
                    // Flywheel off unless launching (manual intake handled above)
                    leftFly.setPower(0);
                    rightFly.setPower(0);

                    if (gamepad1.a) {
                        // Snapshot intake state, then launch takes over intakes
                        intakeWasOnBeforeLaunch = isIntakeOn;
                        isIntakeOn = false;

                        // Start flywheel spinup
                        leftFly.setPower(1.0);
                        rightFly.setPower(1.0);

                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.SPINUP;
                    }
                    break;

                case SPINUP:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);
                    leftFly.setPower(1.0);
                    rightFly.setPower(1.0);

                    if (System.currentTimeMillis() - stateTimer >= FLYWHEEL_SPINUP_MS) {
                        trigger.setPosition(TRIGGER_FIRE);
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.FIRE_FRONT;
                    }
                    break;

                case FIRE_FRONT:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);
                    leftFly.setPower(1.0);
                    rightFly.setPower(1.0);

                    if (System.currentTimeMillis() - stateTimer >= TRIGGER_PULSE_MS) {
                        trigger.setPosition(TRIGGER_HOME);
                        launchState = LaunchState.FEED_FRONT;
                    }
                    break;

                case FEED_FRONT:
                    leftFly.setPower(1.0);
                    rightFly.setPower(1.0);

                    frontIntake.setPower(+LAUNCH_INTAKE_POWER);
                    backIntake.setPower(0);

                    if (centerNewBall) {
                        frontIntake.setPower(0);

                        trigger.setPosition(TRIGGER_FIRE);
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.FIRE_BACK;
                    }
                    break;

                case FIRE_BACK:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);
                    leftFly.setPower(1.0);
                    rightFly.setPower(1.0);

                    if (System.currentTimeMillis() - stateTimer >= TRIGGER_PULSE_MS) {
                        trigger.setPosition(TRIGGER_HOME);
                        launchState = LaunchState.FEED_BACK;
                    }
                    break;

                case FEED_BACK:
                    leftFly.setPower(1.0);
                    rightFly.setPower(1.0);

                    backIntake.setPower(+LAUNCH_INTAKE_POWER);
                    frontIntake.setPower(0);

                    if (centerNewBall) {
                        backIntake.setPower(0);
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.SPINDOWN;
                    }
                    break;

                case SPINDOWN:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);

                    leftFly.setPower(0);
                    rightFly.setPower(0);

                    if (System.currentTimeMillis() - stateTimer >= FLYWHEEL_SPINDOWN_MS) {
                        // Resume intake if it was on before launch
                        isIntakeOn = intakeWasOnBeforeLaunch;
                        launchState = LaunchState.IDLE;
                    }
                    break;
            }

            /* ===================== TELEMETRY ===================== */
            telemetry.addData("LaunchState", launchState);
            telemetry.addData("Turret", "%.3f", currentTurretRotation);
            telemetry.addData("TriggerPos", "%.3f", trigger.getPosition());

            telemetry.addData("Front alpha", "%.2f  ball=%s", f.alpha, frontHasBall);
            telemetry.addData("Center alpha", "%.2f  ball=%s", c.alpha, centerHasBall);
            telemetry.addData("Back alpha", "%.2f  ball=%s", b.alpha, backHasBall);

            telemetry.addData("Intake", "%s | %s",
                    isIntakeOn ? "ON" : "OFF",
                    intakeModeOne ? "MODE1 (IN: FrontFast BackSlow)" : "MODE2 (OUT: BackFast FrontSlow)");
            telemetry.update();
        }
    }

    /* ===================== SMART INTAKE LOGIC ===================== */
    private void applySmartIntake(boolean modeOne) {

        // Decide which side is FAST and which is SLOW based on mode
        // Mode 1 (IN): front FAST, back SLOW
        // Mode 2 (OUT): back FAST, front SLOW (reverse direction)
        DcMotor fastMotor, slowMotor;
        boolean fastHasBall, slowHasBall;

        // Direction: IN = +, OUT = -
        double dir = modeOne ? +1.0 : -1.0;

        if (modeOne) {
            fastMotor = frontIntake;
            slowMotor = backIntake;
            fastHasBall = frontHasBall;
            slowHasBall = backHasBall;
        } else {
            fastMotor = backIntake;
            slowMotor = frontIntake;
            fastHasBall = backHasBall;
            slowHasBall = frontHasBall;
        }

        // 1) Slow motor runs until it "captures" a ball, then stops
        double slowPower = slowHasBall ? 0.0 : (dir * MANUAL_SLOW);

        // 2) Fast motor behavior depends on slow motor capture + center
        double fastPower;

        if (!slowHasBall) {
            // slow not captured yet -> run fast at full
            fastPower = dir * MANUAL_FAST;
        } else {
            // slow captured -> fast slows
            if (centerHasBall) {
                fastPower = dir * FAST_AFTER_SLOW_AND_CENTER;
            } else {
                fastPower = dir * FAST_AFTER_SLOW;
            }

            // 3) Fast motor stops only AFTER slow captured AND center has a ball AND fast position has a ball
            if (centerHasBall && fastHasBall) {
                fastPower = 0.0;
                // shut the whole intake OFF once fully loaded
                isIntakeOn = false;
                slowPower = 0.0;
            }
        }

        // Apply powers to the correct motors
        fastMotor.setPower(fastPower);
        slowMotor.setPower(slowPower);

        // Make sure the "other" motor (the one not fast/slow) isn't accidentally left on.
        // (In this design, every mode always uses both motors, so nothing else to zero here.)
    }

    private boolean hysteresisBall(boolean prev, float alpha) {
        if (prev) {
            // currently "has ball" -> only clear if alpha drops below OFF threshold
            return alpha > ALPHA_OFF;
        } else {
            // currently "no ball" -> set if alpha exceeds ON threshold
            return alpha >= ALPHA_ON;
        }
    }

    /* ===================== HELPERS ===================== */
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

        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake  = hardwareMap.get(DcMotor.class, "backIntake");

        leftFly  = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");

        turretRotation1 = hardwareMap.get(Servo.class, "turret_rotation_1");
        turretRotation2 = hardwareMap.get(Servo.class, "turret_rotation_2");

        trigger = hardwareMap.get(Servo.class, "trigger");

        frontColor  = hardwareMap.get(NormalizedColorSensor.class, "frontColor");
        centerColor = hardwareMap.get(NormalizedColorSensor.class, "centerColor");
        backColor   = hardwareMap.get(NormalizedColorSensor.class, "backColor");

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

        // Flywheel directions (your update)
        leftFly.setDirection(DcMotor.Direction.REVERSE);
        rightFly.setDirection(DcMotor.Direction.FORWARD);

        // Turret servo directions
        turretRotation1.setDirection(Servo.Direction.FORWARD);
        turretRotation2.setDirection(Servo.Direction.REVERSE);

        // Start trigger safely reset/down
        trigger.setPosition(TRIGGER_HOME);
    }
}
