package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "HotPotatoSwerve", group = "Swerve")
public class HotPotatoSwerve extends LinearOpMode {

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

    // Adjuster servo
    private Servo adjuster;

    /* ===================== SENSORS (BALL DISTANCE) ===================== */
    private NormalizedColorSensor frontColor, centerColor, backColor;
    private DistanceSensor frontDist, centerDist, backDist;

    /* ===================== VISION (GOAL DISTANCE) ===================== */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // CenterStage goal IDs (change if yours differ)
    private static final int TAG_BLUE_GOAL = 20;
    private static final int TAG_RED_GOAL  = 24;
    private int targetGoalId = TAG_RED_GOAL;

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

    /* ===================== TURRET CONTROL (GP2 RSX fast, slow with RB) ===================== */
    final double MIN_TURRET_ROTATION = 0.0;
    final double MAX_TURRET_ROTATION = 1.0;
    final double TURRET_DEADBAND = 0.05;

    // “Fast” vs “Slow” sensitivity
    final double TURRET_RATE_FAST = 0.020; // per loop at full stick
    final double TURRET_RATE_SLOW = 0.006; // per loop at full stick

    private double currentTurretRotation = 0.5;

    /* ===================== ADJUSTER SERVO RANGE ===================== */
    final double ADJUSTER_MIN = 0.0;
    final double ADJUSTER_MAX = 0.75;
    private double adjusterPos = ADJUSTER_MIN;

    /* ===================== MANUAL INTAKE CONTROLS ===================== */
    private boolean isIntakeOn = false;
    private boolean intakeTogglePrev = false;

    // Mode 1: IN  -> front FAST, back SLOW
    // Mode 2: OUT -> back  FAST, front SLOW
    private boolean intakeModeOne = true;
    private boolean intakeModePrev = false;

    final double MANUAL_FAST = 1.0;
    final double MANUAL_SLOW = 0.30;

    // Smart gating powers for FAST once slow has captured
    final double FAST_AFTER_SLOW = 0.70;
    final double FAST_AFTER_SLOW_AND_CENTER = 0.55;

    /* ===================== TRIGGER SERVO ===================== */
    // You: 0.0 = launched, 0.225 = down/reset
    final double TRIGGER_FIRE = 0.0;
    final double TRIGGER_HOME = 0.225;

    // Future reference
    final long TRIGGER_PULSE_MS = 250;

    // We hold longer for reliability
    final long LAUNCH_TRIGGER_HOLD_MS = 400;
    final long TRIGGER_RESET_WAIT_MS = 150;

    /* ===================== FLYWHEEL TIMING ===================== */
    final long FLYWHEEL_SPINUP_MS = 1000;
    final long FLYWHEEL_SPINDOWN_MS = 300;

    /* ===================== LAUNCH BEHAVIOR ===================== */
    final long FEED_TIMEOUT_MS = 1000;
    final double LAUNCH_INTAKE_POWER = 0.76;

    /* ===================== DISTANCE BALL DETECTION (INTAKE) ===================== */
    final double FRONT_ON_CM  = 2.2, FRONT_OFF_CM  = 3.2;
    final double CENTER_ON_CM = 2.5, CENTER_OFF_CM = 4.0;
    final double BACK_ON_CM   = 2.5, BACK_OFF_CM   = 4.0;

    private boolean frontHasBall = false;
    private boolean centerHasBall = false;
    private boolean backHasBall = false;

    private boolean centerPrevRaw = false;

    /* ===================== HOTPOTATO: DISTANCE -> SETTINGS LOOKUP ===================== */
    // Distance is from AprilTag pose range (INCHES).
    // Fill these with real tuning data from your robot.
    //
    // Example structure:
    //   distance (in):  30,  45,  60,  75,  90
    //   fly power:     0.60,0.68,0.75,0.82,0.90
    //   adjuster pos:  0.10,0.18,0.26,0.34,0.42
    //
    // Must be same length, sorted by increasing distance.
    private static final double[] DIST_IN =   { 30, 45, 60, 75, 90 };
    private static final double[] FLY_PWR =   { 0.60, 0.68, 0.75, 0.82, 0.90 };
    private static final double[] ADJ_POS =   { 0.10, 0.18, 0.26, 0.34, 0.42 };

    // Current “auto” outputs (applied during launch + can be displayed anytime)
    private double autoFlyPower = 0.75;
    private double autoAdjusterPos = 0.20;

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

        SPINDOWN
    }

    private LaunchState launchState = LaunchState.IDLE;
    private long stateTimer = 0;
    private long feedStartMs = 0;
    private boolean intakeWasOnBeforeLaunch = false;

    @Override
    public void runOpMode() {
        initHardware();
        initVision();

        trigger.setPosition(TRIGGER_HOME);
        turretRotation1.setPosition(currentTurretRotation);
        turretRotation2.setPosition(1.0 - currentTurretRotation);
        adjuster.setPosition(adjusterPos);

        telemetry.addLine("HotPotatoSwerve ready");
        telemetry.addLine("GP1: Drive (RB slow) | Launch A | Abort B | Intake toggle RT | Intake mode dpad_left");
        telemetry.addLine("GP2: Turret RSX (FAST) + hold RB for SLOW | Goal select: dpad_left BLUE, dpad_right RED");
        telemetry.update();

        waitForStart();

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;

        while (opModeIsActive()) {

            /* ===================== GOAL SELECTION (VISION TARGET) ===================== */
            if (gamepad2.dpad_left)  targetGoalId = TAG_BLUE_GOAL;
            if (gamepad2.dpad_right) targetGoalId = TAG_RED_GOAL;

            /* ===================== READ GOAL DISTANCE & COMPUTE AUTO SETTINGS ===================== */
            AprilTagDetection goalDet = getDetectionForId(targetGoalId);
            double goalDistIn = Double.NaN;

            if (goalDet != null && goalDet.ftcPose != null) {
                goalDistIn = goalDet.ftcPose.range; // inches (FTC pose convention)
                autoFlyPower = lookupInterp(DIST_IN, FLY_PWR, goalDistIn);
                autoAdjusterPos = lookupInterp(DIST_IN, ADJ_POS, goalDistIn);

                // clamp adjuster into your physical range
                autoAdjusterPos = clamp(autoAdjusterPos, ADJUSTER_MIN, ADJUSTER_MAX);
                autoFlyPower = clamp(autoFlyPower, 0.0, 1.0);
            }

            /* ===================== BALL DISTANCE SENSING (INTAKE SMART LOGIC) ===================== */
            double fCm = safeDistanceCm(frontDist);
            double cCm = safeDistanceCm(centerDist);
            double bCm = safeDistanceCm(backDist);

            frontHasBall  = hysteresisBall(frontHasBall,  fCm, FRONT_ON_CM,  FRONT_OFF_CM);
            centerHasBall = hysteresisBall(centerHasBall, cCm, CENTER_ON_CM, CENTER_OFF_CM);
            backHasBall   = hysteresisBall(backHasBall,   bCm, BACK_ON_CM,   BACK_OFF_CM);

            boolean centerRaw = (cCm <= CENTER_ON_CM);
            boolean centerNewBall = centerRaw && !centerPrevRaw;
            centerPrevRaw = centerRaw;

            /* ===================== ABORT LAUNCH ANYTIME WITH B ===================== */
            if (gamepad1.b && launchState != LaunchState.IDLE) {
                launchState = LaunchState.IDLE;

                frontIntake.setPower(0);
                backIntake.setPower(0);
                leftFly.setPower(0);
                rightFly.setPower(0);

                trigger.setPosition(TRIGGER_HOME);

                // Resume intake if it was on
                isIntakeOn = intakeWasOnBeforeLaunch;

                telemetry.addLine("LAUNCH ABORTED");
            }

            /* ===================== DRIVE (ROBOT-CENTRIC) ===================== */
            double speedMultiplier = gamepad1.right_bumper ? MAX_SPEED_SLOW : MAX_SPEED_FAST;

            double robotY = -gamepad1.left_stick_y * speedMultiplier;
            double robotX =  gamepad1.left_stick_x * speedMultiplier;
            double rot    =  gamepad1.right_stick_x * speedMultiplier;

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

            if (Math.abs(robotX) > DRIVE_DEADBAND || Math.abs(robotY) > DRIVE_DEADBAND || Math.abs(rot) > DRIVE_DEADBAND) {
                targetAngleFL = Math.atan2(B, D);
                targetAngleFR = Math.atan2(B, C);
                targetAngleBL = Math.atan2(A, D);
                targetAngleBR = Math.atan2(A, C);
            } else {
                spFL = 0; spFR = 0; spBL = 0; spBR = 0;
            }

            runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  spFL, targetAngleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, spFR, targetAngleFR);
            runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   spBL, targetAngleBL);
            runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  spBR, targetAngleBR);

            /* ===================== TURRET MANUAL CONTROL (GP2 RSX, FAST vs SLOW) ===================== */
            double turretInput = -gamepad2.right_stick_x;
            if (Math.abs(turretInput) < TURRET_DEADBAND) turretInput = 0;

            double turretRate = gamepad2.right_bumper ? TURRET_RATE_SLOW : TURRET_RATE_FAST;
            currentTurretRotation += turretInput * turretRate;
            currentTurretRotation = clamp(currentTurretRotation, MIN_TURRET_ROTATION, MAX_TURRET_ROTATION);

            turretRotation1.setPosition(currentTurretRotation);
            turretRotation2.setPosition(1.0 - currentTurretRotation);

            /* ===================== MANUAL INTAKE (ONLY WHEN NOT LAUNCHING) ===================== */
            if (launchState == LaunchState.IDLE) {

                boolean intakeToggle = gamepad1.right_trigger > 0.5;
                if (intakeToggle && !intakeTogglePrev) isIntakeOn = !isIntakeOn;
                intakeTogglePrev = intakeToggle;

                boolean modeToggle = gamepad1.dpad_left;
                if (modeToggle && !intakeModePrev) intakeModeOne = !intakeModeOne;
                intakeModePrev = modeToggle;

                if (isIntakeOn) {
                    applySmartIntake(intakeModeOne);
                } else {
                    frontIntake.setPower(0);
                    backIntake.setPower(0);
                }
            }

            /* ===================== LAUNCH SEQUENCE (GP1 A) ===================== */
            switch (launchState) {

                case IDLE:
                    leftFly.setPower(0);
                    rightFly.setPower(0);

                    if (gamepad1.a) {
                        intakeWasOnBeforeLaunch = isIntakeOn;
                        isIntakeOn = false;

                        // Apply distance-based adjuster & flywheel BEFORE spinup
                        adjusterPos = autoAdjusterPos;
                        adjuster.setPosition(adjusterPos);

                        leftFly.setPower(autoFlyPower);
                        rightFly.setPower(autoFlyPower);

                        trigger.setPosition(TRIGGER_HOME);

                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.SPINUP;
                    }
                    break;

                case SPINUP:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);

                    // Keep holding the tuned setpoints
                    adjuster.setPosition(autoAdjusterPos);
                    leftFly.setPower(autoFlyPower);
                    rightFly.setPower(autoFlyPower);

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

                    adjuster.setPosition(autoAdjusterPos);
                    leftFly.setPower(autoFlyPower);
                    rightFly.setPower(autoFlyPower);

                    if (System.currentTimeMillis() - stateTimer >= LAUNCH_TRIGGER_HOLD_MS) {
                        trigger.setPosition(TRIGGER_HOME);
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.RESET_AFTER_1;
                    }
                    break;

                case RESET_AFTER_1:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);

                    adjuster.setPosition(autoAdjusterPos);
                    leftFly.setPower(autoFlyPower);
                    rightFly.setPower(autoFlyPower);

                    trigger.setPosition(TRIGGER_HOME);

                    if (System.currentTimeMillis() - stateTimer >= TRIGGER_RESET_WAIT_MS) {
                        centerPrevRaw = false;
                        feedStartMs = System.currentTimeMillis();
                        launchState = LaunchState.FEED_FOR_2;
                    }
                    break;

                case FEED_FOR_2:
                    adjuster.setPosition(autoAdjusterPos);
                    leftFly.setPower(autoFlyPower);
                    rightFly.setPower(autoFlyPower);

                    trigger.setPosition(TRIGGER_HOME);

                    // Feed FRONT inward at 76%
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

                    adjuster.setPosition(autoAdjusterPos);
                    leftFly.setPower(autoFlyPower);
                    rightFly.setPower(autoFlyPower);

                    if (System.currentTimeMillis() - stateTimer >= LAUNCH_TRIGGER_HOLD_MS) {
                        trigger.setPosition(TRIGGER_HOME);
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.RESET_AFTER_2;
                    }
                    break;

                case RESET_AFTER_2:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);

                    adjuster.setPosition(autoAdjusterPos);
                    leftFly.setPower(autoFlyPower);
                    rightFly.setPower(autoFlyPower);

                    trigger.setPosition(TRIGGER_HOME);

                    if (System.currentTimeMillis() - stateTimer >= TRIGGER_RESET_WAIT_MS) {
                        centerPrevRaw = false;
                        feedStartMs = System.currentTimeMillis();
                        launchState = LaunchState.FEED_FOR_3;
                    }
                    break;

                case FEED_FOR_3:
                    adjuster.setPosition(autoAdjusterPos);
                    leftFly.setPower(autoFlyPower);
                    rightFly.setPower(autoFlyPower);

                    trigger.setPosition(TRIGGER_HOME);

                    // Feed BACK inward (your previous fix: back intake inward uses negative here)
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

                    adjuster.setPosition(autoAdjusterPos);
                    leftFly.setPower(autoFlyPower);
                    rightFly.setPower(autoFlyPower);

                    if (System.currentTimeMillis() - stateTimer >= LAUNCH_TRIGGER_HOLD_MS) {
                        trigger.setPosition(TRIGGER_HOME);
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.RESET_AFTER_3;
                    }
                    break;

                case RESET_AFTER_3:
                    frontIntake.setPower(0);
                    backIntake.setPower(0);

                    adjuster.setPosition(autoAdjusterPos);
                    leftFly.setPower(autoFlyPower);
                    rightFly.setPower(autoFlyPower);

                    trigger.setPosition(TRIGGER_HOME);

                    if (System.currentTimeMillis() - stateTimer >= TRIGGER_RESET_WAIT_MS) {
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
                        isIntakeOn = intakeWasOnBeforeLaunch;
                        launchState = LaunchState.IDLE;
                    }
                    break;
            }

            /* ===================== TELEMETRY ===================== */
            telemetry.addData("Goal", targetGoalId == TAG_RED_GOAL ? "RED(24)" : "BLUE(20)");
            telemetry.addData("GoalDist(in)", Double.isNaN(goalDistIn) ? "NO TAG" : String.format("%.1f", goalDistIn));
            telemetry.addData("AutoFly", "%.3f", autoFlyPower);
            telemetry.addData("AutoAdjuster", "%.3f", autoAdjusterPos);

            telemetry.addData("DriveMode", gamepad1.right_bumper ? "SLOW" : "FAST");
            telemetry.addData("TurretPos", "%.3f", currentTurretRotation);
            telemetry.addData("TurretRate", gamepad2.right_bumper ? "SLOW" : "FAST");
            telemetry.addData("AdjusterPos(applied)", "%.3f", adjuster.getPosition());

            telemetry.addData("LaunchState", launchState);
            telemetry.addData("TriggerPos", "%.3f", trigger.getPosition());

            telemetry.addData("BallDist F/C/B(cm)", "%.2f / %.2f / %.2f", fCm, cCm, bCm);
            telemetry.addData("Ball F/C/B", "%s / %s / %s", frontHasBall, centerHasBall, backHasBall);

            NormalizedRGBA fr = frontColor.getNormalizedColors();
            NormalizedRGBA ce = centerColor.getNormalizedColors();
            NormalizedRGBA ba = backColor.getNormalizedColors();
            telemetry.addData("RGB Front",  "r=%.2f g=%.2f b=%.2f", fr.red, fr.green, fr.blue);
            telemetry.addData("RGB Center", "r=%.2f g=%.2f b=%.2f", ce.red, ce.green, ce.blue);
            telemetry.addData("RGB Back",   "r=%.2f g=%.2f b=%.2f", ba.red, ba.green, ba.blue);

            telemetry.update();
        }

        if (visionPortal != null) visionPortal.close();
    }

    /* ===================== SMART INTAKE LOGIC ===================== */
    private void applySmartIntake(boolean modeOne) {

        DcMotor fastMotor, slowMotor;
        boolean fastBall, slowBall;
        double dir = modeOne ? +1.0 : -1.0;

        if (modeOne) {
            fastMotor = frontIntake;
            slowMotor = backIntake;
            fastBall = frontHasBall;
            slowBall = backHasBall;
        } else {
            fastMotor = backIntake;
            slowMotor = frontIntake;
            fastBall = backHasBall;
            slowBall = frontHasBall;
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

    /* ===================== DISTANCE HELPERS ===================== */
    private double safeDistanceCm(DistanceSensor s) {
        double cm = s.getDistance(DistanceUnit.CM);
        if (Double.isNaN(cm) || Double.isInfinite(cm)) return 999.0;
        return cm;
    }

    private boolean hysteresisBall(boolean prev, double cm, double onCm, double offCm) {
        return prev ? (cm <= offCm) : (cm <= onCm);
    }

    /* ===================== APRILTAG HELPERS ===================== */
    private AprilTagDetection getDetectionForId(int id) {
        if (aprilTag == null) return null;
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d != null && d.id == id) return d;
        }
        return null;
    }

    /* ===================== LOOKUP TABLE (LINEAR INTERP) ===================== */
    private double lookupInterp(double[] xs, double[] ys, double x) {
        if (xs == null || ys == null || xs.length == 0 || xs.length != ys.length) return 0.0;

        if (x <= xs[0]) return ys[0];
        if (x >= xs[xs.length - 1]) return ys[ys.length - 1];

        for (int i = 0; i < xs.length - 1; i++) {
            double x0 = xs[i], x1 = xs[i + 1];
            if (x >= x0 && x <= x1) {
                double t = (x - x0) / (x1 - x0);
                return ys[i] + t * (ys[i + 1] - ys[i]);
            }
        }
        return ys[ys.length - 1];
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

        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake  = hardwareMap.get(DcMotor.class, "backIntake");

        leftFly  = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");

        turretRotation1 = hardwareMap.get(Servo.class, "turret_rotation_1");
        turretRotation2 = hardwareMap.get(Servo.class, "turret_rotation_2");
        turretRotation1.setDirection(Servo.Direction.FORWARD);
        turretRotation2.setDirection(Servo.Direction.REVERSE);

        trigger = hardwareMap.get(Servo.class, "trigger");
        trigger.setPosition(TRIGGER_HOME);

        adjuster = hardwareMap.get(Servo.class, "adjuster");
        adjuster.setPosition(adjusterPos);

        frontColor  = hardwareMap.get(NormalizedColorSensor.class, "frontColor");
        centerColor = hardwareMap.get(NormalizedColorSensor.class, "centerColor");
        backColor   = hardwareMap.get(NormalizedColorSensor.class, "backColor");

        frontDist  = hardwareMap.get(DistanceSensor.class, "frontColor");
        centerDist = hardwareMap.get(DistanceSensor.class, "centerColor");
        backDist   = hardwareMap.get(DistanceSensor.class, "backColor");

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

        // Your flywheel directions:
        leftFly.setDirection(DcMotor.Direction.REVERSE);
        rightFly.setDirection(DcMotor.Direction.FORWARD);
    }

    private void initVision() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "turretCam"),
                aprilTag
        );
    }
}
