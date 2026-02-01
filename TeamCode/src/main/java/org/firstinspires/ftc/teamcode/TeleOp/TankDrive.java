package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Tank", group = "Swerve")
public class TankDrive extends LinearOpMode {

    /* ===================== DRIVE HARDWARE ===================== */
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;
    private VoltageSensor voltageSensor;

    /* ===================== MECHANISMS ===================== */
    private DcMotor frontIntake, backIntake;
    private DcMotor leftFly, rightFly;

    private Servo turretRotation1, turretRotation2;
    private Servo trigger;
    private Servo adjuster;

    /* ===================== SENSORS (BALL DISTANCE) ===================== */
    private NormalizedColorSensor frontColor, centerColor, backColor; // kept for config compatibility
    private DistanceSensor frontDist, centerDist, backDist;

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

    // Drive speed (GP1 RB slow mode)
    final double MAX_SPEED_FAST = 0.8;
    final double MAX_SPEED_SLOW = 0.2;

    /* ===================== WHEEL PLANTING (X SNAP) ===================== */
    final int FRAMES_TO_PLANT_WHEELS = 5;
    private int framesSinceLastMoved = 0;

    /* ===================== TURRET CONTROL (GP2 RSX, slow with GP2 RB) ===================== */
    final double MIN_TURRET_ROTATION = 0.0;
    final double MAX_TURRET_ROTATION = 1.0;
    final double TURRET_DEADBAND = 0.05;

    final double TURRET_RATE_FAST = 0.030;
    final double TURRET_RATE_SLOW = 0.006;

    private double currentTurretRotation = 0.5;

    // ✅ NEW: edge-detect for GP2 A turret center
    private boolean turretCenterPrev = false;

    /* ===================== ADJUSTER CONTROL (GP2 LSY, slow with GP2 RB, faster than turret) ===================== */
    final double ADJUSTER_MIN = 0.0;   // up
    final double ADJUSTER_MAX = 0.75;  // top
    final double ADJUSTER_DEADBAND = 0.05;

    final double ADJUSTER_RATE_FAST = 0.050;
    final double ADJUSTER_RATE_SLOW = 0.015;

    final double CLOSE_ADJUSTER = 0.433;
    final double FAR_ADJUSTER = 0.118;

    private double adjusterPos = CLOSE_ADJUSTER;

    /* ===================== INTAKES (BASE 90%, SCALED SLOWS) ===================== */
    final double INTAKE_BASE = 0.90;

    final double INTAKE_SLOW_RATIO = 0.30;
    final double FAST_AFTER_SLOW_RATIO = 0.70;
    final double FAST_AFTER_SLOW_AND_CENTER_RATIO = 0.55;

    final double MANUAL_FAST = INTAKE_BASE;
    final double MANUAL_SLOW = INTAKE_BASE * INTAKE_SLOW_RATIO;

    final double FAST_AFTER_SLOW = INTAKE_BASE * FAST_AFTER_SLOW_RATIO;
    final double FAST_AFTER_SLOW_AND_CENTER = INTAKE_BASE * FAST_AFTER_SLOW_AND_CENTER_RATIO;

    final double LAUNCH_INTAKE_POWER = INTAKE_BASE;

    private boolean isIntakeOn = false;
    private boolean intakeTogglePrev = false;

    // Mode 1: IN  -> front FAST, back SLOW
    // Mode 2: OUT -> back  FAST, front SLOW
    private boolean intakeModeOne = true;
    private boolean intakeModePrev = false;

    /* ===================== FLYWHEEL (TOGGLE + TRIM) ===================== */
    private boolean flywheelOn = false;
    private boolean flyTogglePrev = false;

    final double MIN_FLYWHEEL_POWER = 0.690; //0.433
    final double MAX_FLYWHEEL_POWER = 0.830; //0.118

    private double flyPower = MIN_FLYWHEEL_POWER;

    /* ===================== TRIGGER SERVO ===================== */
    // Your mapping: 0.0 = launched, 0.225 = down/reset
    final double TRIGGER_FIRE = 0.0;
    final double TRIGGER_HOME = 0.225;

    final long TRIGGER_PULSE_MS = 250;

    final long LAUNCH_TRIGGER_HOLD_MS = 400;
    final long TRIGGER_RESET_WAIT_MS = 150;

    /* ===================== LAUNCH TIMING ===================== */
    final long FLYWHEEL_SPINUP_MS = 1000;
    final long FLYWHEEL_SPINDOWN_MS = 300;

    final long FEED_TIMEOUT_MS = 1250;

    /* ===================== DISTANCE BALL DETECTION (HYSTERESIS) ===================== */
    final double FRONT_ON_CM  = 2.5, FRONT_OFF_CM  = 3.2;
    final double CENTER_ON_CM = 2.5, CENTER_OFF_CM = 4.0;
    final double BACK_ON_CM   = 2.5, BACK_OFF_CM   = 4.0;

    private boolean frontHasBall = false;
    private boolean centerHasBall = false;
    private boolean backHasBall = false;

    private boolean centerPrevRaw = false;

    /* ===================== CLEAR-CENTER RETRIES (AUTO ABORT) ===================== */
    private static final int MAX_CLEAR_RETRIES = 2;
    private static final long CLEAR_RETRY_GAP_MS = 120;
    private int clearRetryCount = 0;

    private enum ClearTarget { AFTER_1, AFTER_2, BEFORE_SPINDOWN }
    private ClearTarget clearTarget = ClearTarget.AFTER_1;

    /* ===================== LAUNCH STATE MACHINE ===================== */
    private enum LaunchState {
        IDLE,
        SPINUP,

        FIRE_1,
        RESET_AFTER_1,

        FEED_FOR_2,
        FIRE_2,
        RESET_AFTER_2,

        FEED_FOR_3,
        FIRE_3,
        RESET_AFTER_3,

        CLEAR_CENTER,
        CLEAR_PULSE,

        SPINDOWN
    }

    private LaunchState launchState = LaunchState.IDLE;
    private long stateTimer = 0;
    private long feedStartMs = 0;

    @Override
    public void runOpMode() {

        initHardware();

        trigger.setPosition(TRIGGER_HOME);
        turretRotation1.setPosition(currentTurretRotation);
        turretRotation2.setPosition(1.0 - currentTurretRotation);
        adjuster.setPosition(adjusterPos);

        waitForStart();

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;

        while (opModeIsActive()) {

            /* ===================== FLYWHEEL POWER TRIM (GP2 DPAD) ===================== */
            if (gamepad2.dpad_up) {
                flyPower = MAX_FLYWHEEL_POWER;
                adjusterPos = FAR_ADJUSTER;
            } else if (gamepad2.dpad_down) {
                flyPower = MIN_FLYWHEEL_POWER;
                adjusterPos = CLOSE_ADJUSTER;
            }
            flyPower = clamp(flyPower, 0.0, 1.0);

            /* ===================== BALL DISTANCE SENSING ===================== */
            double fCm = safeDistanceCm(frontDist);
            double cCm = safeDistanceCm(centerDist);
            double bCm = safeDistanceCm(backDist);

            frontHasBall  = hysteresisBall(frontHasBall,  fCm, FRONT_ON_CM,  FRONT_OFF_CM);
            centerHasBall = hysteresisBall(centerHasBall, cCm, CENTER_ON_CM, CENTER_OFF_CM);
            backHasBall   = hysteresisBall(backHasBall,   bCm, BACK_ON_CM,   BACK_OFF_CM);

            boolean centerRaw = (cCm <= CENTER_ON_CM);
            boolean centerNewBall = centerRaw && !centerPrevRaw;
            centerPrevRaw = centerRaw;

            /* ===================== DRIVE INPUTS (robot-centric) ===================== */

            double rightTreads = -gamepad1.right_stick_y;
            double leftTreads = -gamepad1.left_stick_y;

            frontRightDrive.setPower(rightTreads);
            backRightDrive.setPower(rightTreads);
            frontLeftDrive.setPower(leftTreads);
            backLeftDrive.setPower(leftTreads);



            /*runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  speedFrontLeft,  targetAngleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR);
            runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   speedBackLeft,   targetAngleBL);
            runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  speedBackRight,  targetAngleBR);

             */

            /* ===================== TURRET CENTER (GP2 A -> 0.5) ===================== */
            boolean centerBtn = gamepad2.a;
            if (centerBtn && !turretCenterPrev) {
                currentTurretRotation = 0.5;
                turretRotation1.setPosition(currentTurretRotation);
                turretRotation2.setPosition(1.0 - currentTurretRotation);
            }
            turretCenterPrev = centerBtn;

            /* ===================== TURRET (GP2 RSX, slow with GP2 RB) ===================== */
            double turretInput = -gamepad2.right_stick_x;
            if (Math.abs(turretInput) < TURRET_DEADBAND) turretInput = 0;

            double turretRate = gamepad2.right_bumper ? TURRET_RATE_SLOW : TURRET_RATE_FAST;
            currentTurretRotation = clamp(currentTurretRotation + turretInput * turretRate,
                    MIN_TURRET_ROTATION, MAX_TURRET_ROTATION);

            turretRotation1.setPosition(currentTurretRotation);
            turretRotation2.setPosition(1.0 - currentTurretRotation);

            /* ===================== ADJUSTER (GP2 LSY, slow with GP2 RB) ===================== */
            double adjInput = gamepad2.left_stick_y;
            if (Math.abs(adjInput) < ADJUSTER_DEADBAND) adjInput = 0;

            double adjRate = gamepad2.right_bumper ? ADJUSTER_RATE_SLOW : ADJUSTER_RATE_FAST;
            adjusterPos = clamp(adjusterPos + adjInput * adjRate, ADJUSTER_MIN, ADJUSTER_MAX);//*/
            adjuster.setPosition(adjusterPos);

            /* ===================== MANUAL INTAKE (when not launching) ===================== */
            if (launchState == LaunchState.IDLE) {
                boolean intakeToggle = gamepad1.right_trigger > 0.5;
                if (intakeToggle && !intakeTogglePrev) isIntakeOn = !isIntakeOn;
                intakeTogglePrev = intakeToggle;

                boolean modeToggle = gamepad1.dpad_left;
                if (modeToggle && !intakeModePrev) intakeModeOne = !intakeModeOne;
                intakeModePrev = modeToggle;

                if (isIntakeOn) applySmartIntake();
                else { frontIntake.setPower(0); backIntake.setPower(0); }
            }

            /* ===================== FLYWHEEL TOGGLE (GP1 LT) ===================== */
           /* boolean flyToggle = gamepad1.left_trigger > 0.5;
            if (flyToggle && !flyTogglePrev) flywheelOn = !flywheelOn;
            flyTogglePrev = flyToggle;

            if (launchState == LaunchState.IDLE) {
                if (flywheelOn) {
                    leftFly.setPower(flyPower);
                    rightFly.setPower(flyPower);
                } else {
                    leftFly.setPower(0);
                    rightFly.setPower(0);
                }
            }*/

            /* ===================== LAUNCH START (GP1 A) ===================== */
            if (launchState == LaunchState.IDLE && gamepad1.a) {
                leftFly.setPower(flyPower);
                rightFly.setPower(flyPower);

                isIntakeOn = false;
                frontIntake.setPower(0);
                backIntake.setPower(0);

                trigger.setPosition(TRIGGER_HOME);

                clearRetryCount = 1;
                stateTimer = System.currentTimeMillis();
                launchState = LaunchState.SPINUP;
            }

            /* ===================== LAUNCH ABORT (GP1 B) ===================== */
            if (gamepad1.b && launchState != LaunchState.IDLE) {
                abortLaunch();
            }

            /* ===================== LAUNCH SEQUENCE ===================== */
            switch (launchState) {

                case IDLE:
                    break;

                case SPINUP:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);
                    trigger.setPosition(TRIGGER_HOME);

                    if (System.currentTimeMillis() - stateTimer >= FLYWHEEL_SPINUP_MS) {
                        trigger.setPosition(TRIGGER_FIRE);
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.FIRE_1;
                    }
                    break;

                case FIRE_1:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);

                    if (System.currentTimeMillis() - stateTimer >= LAUNCH_TRIGGER_HOLD_MS) {
                        trigger.setPosition(TRIGGER_HOME);
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.RESET_AFTER_1;
                    }
                    break;

                case RESET_AFTER_1:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);
                    trigger.setPosition(TRIGGER_HOME);

                    if (System.currentTimeMillis() - stateTimer >= TRIGGER_RESET_WAIT_MS) {
                        clearTarget = ClearTarget.AFTER_1;
                        clearRetryCount = 1;
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.CLEAR_CENTER;
                    }
                    break;

                case FEED_FOR_2:
                    trigger.setPosition(TRIGGER_HOME);

                    frontIntake.setPower(+LAUNCH_INTAKE_POWER);
                    backIntake.setPower(0);

                    if (centerNewBall || (System.currentTimeMillis() - feedStartMs >= FEED_TIMEOUT_MS)) {
                        frontIntake.setPower(0);
                        backIntake.setPower(0);

                        trigger.setPosition(TRIGGER_FIRE);
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.FIRE_2;
                    }
                    break;

                case FIRE_2:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);

                    if (System.currentTimeMillis() - stateTimer >= LAUNCH_TRIGGER_HOLD_MS) {
                        trigger.setPosition(TRIGGER_HOME);
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.RESET_AFTER_2;
                    }
                    break;

                case RESET_AFTER_2:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);
                    trigger.setPosition(TRIGGER_HOME);

                    if (System.currentTimeMillis() - stateTimer >= TRIGGER_RESET_WAIT_MS) {
                        clearTarget = ClearTarget.AFTER_2;
                        clearRetryCount = 1;
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.CLEAR_CENTER;
                    }
                    break;

                case FEED_FOR_3:
                    trigger.setPosition(TRIGGER_HOME);

                    backIntake.setPower(-LAUNCH_INTAKE_POWER);
                    frontIntake.setPower(0);

                    if (centerNewBall || (System.currentTimeMillis() - feedStartMs >= FEED_TIMEOUT_MS)) {
                        backIntake.setPower(0);
                        frontIntake.setPower(0);

                        trigger.setPosition(TRIGGER_FIRE);
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.FIRE_3;
                    }
                    break;

                case FIRE_3:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);

                    if (System.currentTimeMillis() - stateTimer >= LAUNCH_TRIGGER_HOLD_MS) {
                        trigger.setPosition(TRIGGER_HOME);
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.RESET_AFTER_3;
                    }
                    break;

                case RESET_AFTER_3:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);
                    trigger.setPosition(TRIGGER_HOME);

                    if (System.currentTimeMillis() - stateTimer >= TRIGGER_RESET_WAIT_MS) {
                        clearTarget = ClearTarget.BEFORE_SPINDOWN;
                        clearRetryCount = 1;
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.CLEAR_CENTER;
                    }
                    break;

                case CLEAR_CENTER:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);
                    trigger.setPosition(TRIGGER_HOME);

                    if (isCenterEmpty(cCm)) {
                        clearRetryCount = 1;
                        centerPrevRaw = false;
                        feedStartMs = System.currentTimeMillis();

                        if (clearTarget == ClearTarget.AFTER_1) {
                            launchState = LaunchState.FEED_FOR_2;
                        } else if (clearTarget == ClearTarget.AFTER_2) {
                            launchState = LaunchState.FEED_FOR_3;
                        } else {
                            stateTimer = System.currentTimeMillis();
                            launchState = LaunchState.SPINDOWN;
                        }
                        break;
                    }

                    if (clearRetryCount >= MAX_CLEAR_RETRIES) {
                        abortLaunch();
                        break;
                    }

                    if (System.currentTimeMillis() - stateTimer >= CLEAR_RETRY_GAP_MS) {
                        trigger.setPosition(TRIGGER_FIRE);
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.CLEAR_PULSE;
                    }
                    break;

                case CLEAR_PULSE:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);

                    if (System.currentTimeMillis() - stateTimer >= LAUNCH_TRIGGER_HOLD_MS) {
                        trigger.setPosition(TRIGGER_HOME);
                        clearRetryCount++;

                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.CLEAR_CENTER;
                    }
                    break;

                case SPINDOWN:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);
                    trigger.setPosition(TRIGGER_HOME);

                    leftFly.setPower(0);
                    rightFly.setPower(0);

                    if (System.currentTimeMillis() - stateTimer >= FLYWHEEL_SPINDOWN_MS) {
                        launchState = LaunchState.IDLE;
                    }
                    break;
            }

            /* ===================== TELEMETRY (includes fly trim + voltage) ===================== */
            double vbat = (voltageSensor != null) ? voltageSensor.getVoltage() : Double.NaN;
            telemetry.addData("FlyPower", "%.3f", flyPower);
            telemetry.addData("VBatt", Double.isNaN(vbat) ? "?" : String.format("%.2fV", vbat));
            telemetry.addData("Turret", "%.3f", currentTurretRotation);
            telemetry.addData("Adjuster", "%.3f", adjusterPos);
            telemetry.update();
        }
    }

    /* ===================== SMART INTAKE LOGIC ===================== */
    private void applySmartIntake() {

        DcMotor fastMotor, slowMotor;
        boolean fastBall, slowBall;

        double dir = intakeModeOne ? +1.0 : -1.0;

        if (intakeModeOne) {
            fastMotor = frontIntake;  fastBall = frontHasBall;
            slowMotor = backIntake;   slowBall = backHasBall;
        } else {
            fastMotor = backIntake;   fastBall = backHasBall;
            slowMotor = frontIntake;  slowBall = frontHasBall;
        }

        double slowPower = slowBall ? 0.0 : (dir * MANUAL_SLOW);

        double fastPower;
        if (!slowBall) {
            fastPower = dir * MANUAL_FAST;
        } else {
            fastPower = dir * (centerHasBall ? FAST_AFTER_SLOW_AND_CENTER : FAST_AFTER_SLOW);

            if (centerHasBall && fastBall) {
                fastPower = 0.0;
                slowPower = 0.0;
                isIntakeOn = false;
            }
        }

        fastMotor.setPower(fastPower);
        slowMotor.setPower(slowPower);
    }

    /* ===================== CLEAR/ABORT HELPERS ===================== */
    private boolean isCenterEmpty(double centerCm) {
        return centerCm >= CENTER_OFF_CM;
    }

    private void abortLaunch() {
        frontIntake.setPower(0);
        backIntake.setPower(0);
        leftFly.setPower(0);
        rightFly.setPower(0);
        trigger.setPosition(TRIGGER_HOME);

        clearRetryCount = 0;
        launchState = LaunchState.IDLE;
    }

    /* ===================== DISTANCE HELPERS ===================== */
    private double safeDistanceCm(DistanceSensor s) {
        double cm = s.getDistance(DistanceUnit.CM);
        if (Double.isNaN(cm) || Double.isInfinite(cm)) return 999.0;
        return cm;
    }

    private boolean hysteresisBall(boolean prev, double cm, double onCm, double offCm) {
        return prev ? (cm <= offCm) : (cm <= onCm);
    }

    /* ===================== SWERVE HELPERS ===================== */
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

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /* ===================== HARDWARE INIT ===================== */
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

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

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

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake  = hardwareMap.get(DcMotor.class, "backIntake");

        leftFly  = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");
        leftFly.setDirection(DcMotor.Direction.REVERSE);
        rightFly.setDirection(DcMotor.Direction.FORWARD);

        turretRotation1 = hardwareMap.get(Servo.class, "turret_rotation_1");
        turretRotation2 = hardwareMap.get(Servo.class, "turret_rotation_2");
        turretRotation1.setDirection(Servo.Direction.FORWARD);
        turretRotation2.setDirection(Servo.Direction.REVERSE);

        trigger = hardwareMap.get(Servo.class, "trigger");
        //trigger.setPosition(TRIGGER_HOME);

        adjuster = hardwareMap.get(Servo.class, "adjuster");
        //adjuster.setPosition(adjusterPos);

        frontColor  = hardwareMap.get(NormalizedColorSensor.class, "frontColor");
        centerColor = hardwareMap.get(NormalizedColorSensor.class, "centerColor");
        backColor   = hardwareMap.get(NormalizedColorSensor.class, "backColor");

        frontDist  = hardwareMap.get(DistanceSensor.class, "frontColor");
        centerDist = hardwareMap.get(DistanceSensor.class, "centerColor");
        backDist   = hardwareMap.get(DistanceSensor.class, "backColor");
    }
}