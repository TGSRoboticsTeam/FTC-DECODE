package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// ===== Vision / AprilTag =====
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp(name = "loadedHotPotatoSwerve", group = "Swerve")
public class loadedHotPotatoSwerve extends LinearOpMode {

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

    // >>> LIGHT SERVO <<<
    private Servo light;

    /* ===================== LIGHT SERVO POSITIONS ===================== */
    private static final double LIGHT_NO_TAG     = 0.0;   // no tag visible
    private static final double LIGHT_TAG_SEEN   = 0.388; // tag visible, not locked
    private static final double LIGHT_TAG_LOCKED = 0.5;   // within 3 degrees of image center

    // "Locked" is based on TAG CENTER in the image (not ftcPose yaw/bearing)
    private static final double LIGHT_LOCK_DEG = 3.0;

    // Tune these for your VisionPortal resolution + your webcam HFOV
    // If your lock feels too strict/loose, adjust CAM_HFOV_DEG first.
    private static final int TAG_FRAME_WIDTH_PX = 640;  // common: 640 (set to your actual frame width)
    private static final double CAM_HFOV_DEG = 60.0;    // horizontal FOV in degrees (tune for your webcam)

    /* ===================== SIDE SORT SERVO ===================== */
    private Servo sideSort;
    private static final double SIDE_SORT_CENTERED = 0.4;
    private static final double SIDE_SORT_STOWED   = 0.925;
    private double sideSortPos = SIDE_SORT_CENTERED;
    private boolean sideSortTogglePrev = false;

    /* ===================== MULTIPLIER (AUTO FROM CAMERA) ===================== */
    private double multiplier = 1.0;

    /* ===================== SENSORS (BALL DISTANCE) ===================== */
    private NormalizedColorSensor frontColor, centerColor, backColor; // config compatibility
    private DistanceSensor frontDist, centerDist, backDist;

    /* ===================== VOLTAGE COMP (FLYWHEEL) ===================== */
    private static final double REF_VOLTAGE = 12.0;

    /* ===================== VOLTAGE COMP (INTAKES) ===================== */
    private static final double INTAKE_REF_VOLTAGE = 10.0;

    /** Returns a multiplier that makes intakes behave like "power = 1.0 at 10V", clamped to 1.0. */
    private double intakeVoltageScale(double vbat) {
        if (Double.isNaN(vbat) || Double.isInfinite(vbat) || vbat <= 0.5) return 1.0;
        return clamp(INTAKE_REF_VOLTAGE / vbat, 0.0, 1.0);
    }

    /** Apply intake voltage scaling to any desired intake power. */
    private double intakeCommand(double desiredPower, double vbat) {
        return clamp(desiredPower * intakeVoltageScale(vbat), -1.0, 1.0);
    }

    /* ===================== SWERVE CONSTANTS ===================== */
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    final double FRONT_LEFT_OFFSET  = 1.34;
    final double FRONT_RIGHT_OFFSET = 3.161;
    final double BACK_LEFT_OFFSET   = 1.589;
    final double BACK_RIGHT_OFFSET  = 1.237;

    final double STEER_KP = 0.8;
    final double DRIVE_DEADBAND = 0.05;
    final double STEER_DEADBAND = 0.08;

    // Drive speed (GP1 RB slow mode)
    final double MAX_SPEED_FAST = 0.8;
    final double MAX_SPEED_SLOW = 0.4;

    /* ===================== WHEEL PLANTING (X SNAP) ===================== */
    final int FRAMES_TO_PLANT_WHEELS = 5;
    private int framesSinceLastMoved = 0;

    /* ===================== TURRET CONTROL (RIGHT STICK ONLY, SMOOTH; RB = SLOW) ===================== */
    final double MIN_TURRET_ROTATION = 0.0;
    final double MAX_TURRET_ROTATION = 1.0;
    final double TURRET_DEADBAND = 0.05;

    // Tuning knobs:
    final double TURRET_RATE_FAST = 0.045;   // faster when RB not held
    final double TURRET_RATE_SLOW = 0.012;   // slower when RB held
    final double TURRET_SMOOTHING = 0.25;    // 0..1 (higher = more responsive, lower = smoother)

    private double turretSmoothedInput = 0.0;
    private double currentTurretRotation = 0.5;
    private boolean turretCenterPrev = false;

    /* ===================== ADJUSTER LIMITS (AUTO POS FROM TABLE) ===================== */
    final double ADJUSTER_MIN = 0.0;   // up
    final double ADJUSTER_MAX = 0.75;  // top
    private double adjusterPos = 0.4482;

    /* ===================== INTAKES (BASE 90%, SCALED SLOWS) ===================== */
    final double INTAKE_BASE = 0.90;

    final double INTAKE_SLOW_RATIO = 0.30;
    final double FAST_AFTER_SLOW_RATIO = 0.70;
    final double FAST_AFTER_SLOW_AND_CENTER_RATIO = 0.625;

    final double MANUAL_FAST = INTAKE_BASE;
    final double MANUAL_SLOW = INTAKE_BASE * INTAKE_SLOW_RATIO;

    final double FAST_AFTER_SLOW = INTAKE_BASE * FAST_AFTER_SLOW_RATIO;
    final double FAST_AFTER_SLOW_AND_CENTER = INTAKE_BASE * FAST_AFTER_SLOW_AND_CENTER_RATIO;

    // 7% slower when 2 balls are present (slow slot occupied)
    private static final double TWO_BALL_SLOW_FACTOR = 0.93;

    private boolean isIntakeOn = false;
    private boolean intakeTogglePrev = false;

    private boolean dpadUpPrev = false;

    // Mode 1: IN  -> front FAST, back SLOW
    // Mode 2: OUT -> back  FAST, front SLOW
    private boolean intakeModeOne = true;
    private boolean intakeModePrev = false;

    /* ===================== LAUNCH FEEDING (TUNING) ===================== */
    private static final double FRONT_FEED_POWER = 0.65;
    private static final double BACK_FEED_POWER  = 0.50;
    private static final long FEED_TO_CENTER_MS = 450;

    /* ===================== OUTWARD BURP (ONLY WHEN LAUNCH STARTS) ===================== */
    private static final double SPINUP_OUTWARD_POWER = 0.4;
    private static final long SPINUP_OUTWARD_MS = 30;

    /* ===================== TRIGGER SERVO ===================== */
    final double TRIGGER_FIRE = 0.0;
    final double TRIGGER_HOME = 0.225;

    final long LAUNCH_TRIGGER_HOLD_MS = 290;
    final long TRIGGER_RESET_WAIT_MS = 125;
    final long RETRY_EXTRA_WAIT_MS = 300;

    /* ===================== LAUNCH TIMING ===================== */
    final long FLYWHEEL_SPINUP_MS = 1000;
    final long FLYWHEEL_SPINDOWN_MS = 300;

    /* ===================== DISTANCE BALL DETECTION (HYSTERESIS) ===================== */
    final double FRONT_ON_CM  = 4.0, FRONT_OFF_CM  = 5.0;
    final double CENTER_ON_CM = 4.0, CENTER_OFF_CM = 5.0;
    final double BACK_ON_CM   = 4.0, BACK_OFF_CM   = 5.0;

    private boolean frontHasBall = false;
    private boolean centerHasBall = false;
    private boolean backHasBall = false;

    /* ===================== RETRY CONTROL ===================== */
    private static final int MAX_RETRIES_PER_SHOT = 3;
    private int shotRetryCount = 0;

    /* ===================== SNAPSHOT PLANS (captured at A press) ===================== */
    private boolean planCenterShot = false;
    private boolean planFrontShot  = false;
    private boolean planBackShot   = false;

    /* ===================== FLYWHEEL PRE-SPIN TOGGLE (GP1 LEFT TRIGGER TAP) ===================== */
    private boolean preSpinLatched = false;
    private boolean ltPrev = false;
    private long preSpinStartMs = 0;

    /* ===================== AUTO PRESPIN WHEN INTAKES AUTO-STOP ===================== */
    private static final boolean AUTO_PRESPIN_WHEN_FULL = true;

    /* ===================== FULL-STACK LATCH (set when smart intake auto-stops) ===================== */
    private boolean fullStackLatched = false;

    /* ===================== SPINUP REMAINING (after BURP) ===================== */
    private long spinupRemainingMs = 0;

    /* ===================== LAUNCH STATE MACHINE ===================== */
    private enum LaunchState {
        IDLE,
        BURP,
        SPINUP,

        FIRE_1,
        RESET_AFTER_1,

        FEED_2,
        FIRE_2,
        RESET_AFTER_2,

        FEED_3,
        FIRE_3,
        RESET_AFTER_3,

        SPINDOWN
    }

    private LaunchState launchState = LaunchState.IDLE;
    private long stateTimer = 0;

    /* ===================== VISION (turretCam + AprilTag) ===================== */
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // ===================== UPDATED AUTO FIT FROM TABLE (distance inches -> multiplier) =====================
    private static final double AUTO_M = 0.0022636364;
    private static final double AUTO_B = 0.5415909091;
    private static final double AUTO_ADJUSTER_POS = 0.4482;

    // Hold-last behavior when tag not visible
    private double lastKnownMultiplier = 0.66;
    private double lastKnownAdjusterPos = AUTO_ADJUSTER_POS;

    private void initTurretCamAprilTags() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "turretCam"))
                .addProcessor(aprilTag)
                .build();
    }

    private AprilTagDetection getTag24() {
        if (aprilTag == null) return null;
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection d : detections) {
            if (d != null && d.id == 24 && d.ftcPose != null) return d;
        }
        return null;
    }

    private double getTag24RangeInches() {
        AprilTagDetection d = getTag24();
        return (d == null) ? Double.NaN : d.ftcPose.range; // inches
    }

    // Still useful for debugging even though light "lock" is based on image centering
    private double getTag24BearingDegrees() {
        AprilTagDetection d = getTag24();
        return (d == null) ? Double.NaN : d.ftcPose.bearing; // degrees
    }

    private double getTagCenterOffsetDeg(AprilTagDetection d) {
        if (d == null) return Double.NaN;
        // Tag center in pixels -> degrees from image center using HFOV
        double dxPx = d.center.x - (TAG_FRAME_WIDTH_PX / 2.0);
        return (dxPx * CAM_HFOV_DEG) / TAG_FRAME_WIDTH_PX;
    }

    private void updateLightServoFromCentering(AprilTagDetection d) {
        if (light == null) return;

        if (d == null) {
            light.setPosition(LIGHT_NO_TAG);
            return;
        }

        double degFromCenter = getTagCenterOffsetDeg(d);
        if (!Double.isNaN(degFromCenter) && Math.abs(degFromCenter) <= LIGHT_LOCK_DEG) {
            light.setPosition(LIGHT_TAG_LOCKED);
        } else {
            light.setPosition(LIGHT_TAG_SEEN);
        }
    }

    private void telemetryTag24(double tagRangeIn, double bearingDeg, double centerOffsetDeg) {
        if (Double.isNaN(tagRangeIn)) {
            telemetry.addLine("Tag 24: NOT VISIBLE");
        } else {
            telemetry.addLine("Tag 24: FOUND");
            telemetry.addData("Tag24 Dist (in)", "%.2f", tagRangeIn);
            telemetry.addData("Tag24 Bearing (deg)", "%.2f", bearingDeg);
            telemetry.addData("Tag Center Offset (deg)", "%.2f", centerOffsetDeg);
        }
    }

    /* ===================== BALL COLOR CLASSIFICATION ===================== */
    // Rule you gave:
    // - if green > (red + blue) => GREEN
    // - if (red + blue) > green => PURPLE
    private String classifyBallColor(NormalizedColorSensor s) {
        if (s == null) return "n/a";
        NormalizedRGBA c = s.getNormalizedColors();
        double r = c.red;
        double g = c.green;
        double b = c.blue;

        if (g > (r + b)) return "GREEN";
        if ((r + b) > g) return "PURPLE";
        return "UNKNOWN";
    }

    @Override
    public void runOpMode() {

        initHardware();
        initTurretCamAprilTags();

        if (trigger != null) trigger.setPosition(TRIGGER_HOME);
        if (turretRotation1 != null) turretRotation1.setPosition(currentTurretRotation);
        if (turretRotation2 != null) turretRotation2.setPosition(1.0 - currentTurretRotation);

        adjusterPos = lastKnownAdjusterPos;
        if (adjuster != null) adjuster.setPosition(adjusterPos);

        if (sideSort != null) {
            sideSortPos = SIDE_SORT_CENTERED;
            sideSort.setPosition(sideSortPos);
        }

        if (light != null) light.setPosition(LIGHT_NO_TAG);

        waitForStart();

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;

        while (opModeIsActive()) {

            /* ===================== SIDE SORT TOGGLE (GP2 X) ===================== */
            boolean sideSortToggle = gamepad2.x;
            if (sideSortToggle && !sideSortTogglePrev) {
                sideSortPos = (sideSortPos == SIDE_SORT_CENTERED) ? SIDE_SORT_STOWED : SIDE_SORT_CENTERED;
                if (sideSort != null) sideSort.setPosition(sideSortPos);
            }
            sideSortTogglePrev = sideSortToggle;

            /* ===================== BALL DISTANCE SENSING ===================== */
            double fCm = safeDistanceCm(frontDist);
            double cCm = safeDistanceCm(centerDist);
            double bCm = safeDistanceCm(backDist);

            frontHasBall  = hysteresisBall(frontHasBall,  fCm, FRONT_ON_CM,  FRONT_OFF_CM);
            centerHasBall = hysteresisBall(centerHasBall, cCm, CENTER_ON_CM, CENTER_OFF_CM);
            backHasBall   = hysteresisBall(backHasBall,   bCm, BACK_ON_CM,   BACK_OFF_CM);

            /* ===================== DRIVE SPEED MODE ===================== */
            double speedMultiplier = gamepad1.right_bumper ? MAX_SPEED_SLOW : MAX_SPEED_FAST;

            // --- Field-Centric Reset (Gamepad 1 dpad_up) ---
            boolean dpadUpNow = gamepad1.dpad_up;
            if (dpadUpNow && !dpadUpPrev) {
                telemetry.addLine("Resetting field heading");
                telemetry.update();
                imu.resetYaw();
            }
            dpadUpPrev = dpadUpNow;

            /* ===================== DRIVE INPUTS (field-centric) ===================== */
            double fieldY = -gamepad1.left_stick_y * speedMultiplier; // forward
            double fieldX =  gamepad1.left_stick_x * speedMultiplier; // strafe
            double rot    =  gamepad1.right_stick_x * speedMultiplier; // rotation

            double rawHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double botHeading = wrapAngle(rawHeading);

            // Convert field -> robot (rotate by -heading)
            double robotX = fieldX * Math.cos(-botHeading) - fieldY * Math.sin(-botHeading);
            double robotY = fieldX * Math.sin(-botHeading) + fieldY * Math.cos(-botHeading);

            double A = robotX - rot * (WHEELBASE / R);
            double B = robotX + rot * (WHEELBASE / R);
            double C = robotY - rot * (TRACK_WIDTH / R);
            double D = robotY + rot * (TRACK_WIDTH / R);

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

            // Steering targets
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

            // X formation plant
            if (gamepad1.left_stick_button || framesSinceLastMoved >= FRAMES_TO_PLANT_WHEELS) {
                targetAngleFL = -Math.PI / 4; targetAngleFR =  Math.PI / 4;
                targetAngleBL =  Math.PI / 4; targetAngleBR = -Math.PI / 4;
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
            }

            runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  speedFrontLeft,  targetAngleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR);
            runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   speedBackLeft,   targetAngleBL);
            runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  speedBackRight,  targetAngleBR);

            /* ===================== TURRET CENTER (GP2 A -> 0.5) ===================== */
            boolean centerBtn = gamepad2.a;
            if (centerBtn && !turretCenterPrev) {
                currentTurretRotation = 0.5;
                turretSmoothedInput = 0.0;
                if (turretRotation1 != null) turretRotation1.setPosition(currentTurretRotation);
                if (turretRotation2 != null) turretRotation2.setPosition(1.0 - currentTurretRotation);
            }
            turretCenterPrev = centerBtn;

            /* ===================== APRILTAG: MULTIPLIER + ADJUSTER + LIGHT ===================== */
            AprilTagDetection d24 = getTag24();
            double tagRangeIn = getTag24RangeInches();
            double bearingDeg = getTag24BearingDegrees();
            double centerOffsetDeg = getTagCenterOffsetDeg(d24);
            boolean tagVisible = !Double.isNaN(tagRangeIn);

            // Light rule you wanted: based on tag center being within +/- 3 degrees of the image center
            updateLightServoFromCentering(d24);

            if (tagVisible) {
                multiplier = clamp(AUTO_M * tagRangeIn + AUTO_B, 0.0, 1.0);
                adjusterPos = clamp(AUTO_ADJUSTER_POS, ADJUSTER_MIN, ADJUSTER_MAX);
                lastKnownMultiplier = multiplier;
                lastKnownAdjusterPos = adjusterPos;
                if (adjuster != null) adjuster.setPosition(adjusterPos);
            } else {
                multiplier = clamp(lastKnownMultiplier, 0.0, 1.0);
                adjusterPos = clamp(lastKnownAdjusterPos, ADJUSTER_MIN, ADJUSTER_MAX);
                if (adjuster != null) adjuster.setPosition(adjusterPos);
            }

            /* ===================== FLYWHEEL COMMAND ===================== */
            double vbat = (voltageSensor != null) ? voltageSensor.getVoltage() : Double.NaN;
            double flyCmd = flywheelCommand(vbat);

            /* ===================== TURRET (RIGHT STICK ONLY, SMOOTH; RB = SLOW) ===================== */
            double rawInput = -gamepad2.right_stick_x;
            if (Math.abs(rawInput) < TURRET_DEADBAND) rawInput = 0.0;

            turretSmoothedInput = turretSmoothedInput + TURRET_SMOOTHING * (rawInput - turretSmoothedInput);
            double turretRate = gamepad2.right_bumper ? TURRET_RATE_SLOW : TURRET_RATE_FAST;

            currentTurretRotation = clamp(
                    currentTurretRotation + turretSmoothedInput * turretRate,
                    MIN_TURRET_ROTATION,
                    MAX_TURRET_ROTATION
            );

            if (turretRotation1 != null) turretRotation1.setPosition(currentTurretRotation);
            if (turretRotation2 != null) turretRotation2.setPosition(1.0 - currentTurretRotation);

            /* ===================== PRE-SPIN TOGGLE (GP1 LEFT TRIGGER TAP) ===================== */
            boolean ltNow = (gamepad1.left_trigger > 0.5);
            boolean ltPressedEdge = ltNow && !ltPrev;
            ltPrev = ltNow;

            if (launchState == LaunchState.IDLE && ltPressedEdge) {
                preSpinLatched = !preSpinLatched;
                if (preSpinLatched) preSpinStartMs = System.currentTimeMillis();
                else {
                    preSpinStartMs = 0;
                    if (leftFly != null) leftFly.setPower(0);
                    if (rightFly != null) rightFly.setPower(0);
                }
            }

            if (preSpinLatched && launchState == LaunchState.IDLE) {
                if (leftFly != null) leftFly.setPower(flyCmd);
                if (rightFly != null) rightFly.setPower(flyCmd);

                if (frontIntake != null) frontIntake.setPower(0);
                if (backIntake != null) backIntake.setPower(0);

                if (trigger != null) trigger.setPosition(TRIGGER_HOME);
            }

            /* ===================== MANUAL INTAKE (when not launching) ===================== */
            if (launchState == LaunchState.IDLE) {
                boolean intakeToggle = gamepad1.right_trigger > 0.5;
                if (intakeToggle && !intakeTogglePrev) isIntakeOn = !isIntakeOn;
                intakeTogglePrev = intakeToggle;

                boolean modeToggle = gamepad1.dpad_left;
                if (modeToggle && !intakeModePrev) intakeModeOne = !intakeModeOne;
                intakeModePrev = modeToggle;

                if (!preSpinLatched) {
                    if (isIntakeOn) applySmartIntake(vbat, flyCmd);
                    else {
                        if (frontIntake != null) frontIntake.setPower(0);
                        if (backIntake != null) backIntake.setPower(0);
                    }
                }
            }

            /* ===================== LAUNCH START (GP1 A) ===================== */
            if (launchState == LaunchState.IDLE && gamepad1.a) {

                boolean runAllThree = fullStackLatched;

                planCenterShot = true; // always attempt center first
                planFrontShot  = runAllThree || frontHasBall;
                planBackShot   = runAllThree || backHasBall;

                long now = System.currentTimeMillis();
                long preSpinElapsed = (preSpinStartMs > 0) ? (now - preSpinStartMs) : 0;
                spinupRemainingMs = Math.max(0, FLYWHEEL_SPINUP_MS - preSpinElapsed);

                preSpinLatched = false;
                preSpinStartMs = 0;
                fullStackLatched = false;

                if (leftFly != null) leftFly.setPower(flyCmd);
                if (rightFly != null) rightFly.setPower(flyCmd);

                isIntakeOn = false;
                if (frontIntake != null) frontIntake.setPower(0);
                if (backIntake != null) backIntake.setPower(0);

                if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                shotRetryCount = 0;

                stateTimer = now;
                launchState = LaunchState.BURP;
            }

            /* ===================== LAUNCH ABORT (GP1 B) ===================== */
            if (gamepad1.b && launchState != LaunchState.IDLE) {
                abortLaunch();
            }

            /* ===================== LAUNCH SEQUENCE ===================== */
            switch (launchState) {

                case IDLE:
                    break;

                case BURP: {
                    if (leftFly != null) leftFly.setPower(flyCmd);
                    if (rightFly != null) rightFly.setPower(flyCmd);

                    if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                    if (frontIntake != null) frontIntake.setPower(intakeCommand(-SPINUP_OUTWARD_POWER, vbat));
                    if (backIntake != null)  backIntake.setPower(intakeCommand(+SPINUP_OUTWARD_POWER, vbat));

                    if (System.currentTimeMillis() - stateTimer >= SPINUP_OUTWARD_MS) {
                        if (frontIntake != null) frontIntake.setPower(0);
                        if (backIntake != null) backIntake.setPower(0);

                        stateTimer = System.currentTimeMillis();

                        if (spinupRemainingMs > 0) {
                            launchState = LaunchState.SPINUP;
                        } else {
                            if (trigger != null) trigger.setPosition(TRIGGER_FIRE);
                            shotRetryCount = 0;
                            launchState = LaunchState.FIRE_1;
                        }
                    }
                    break;
                }

                case SPINUP: {
                    if (leftFly != null) leftFly.setPower(flyCmd);
                    if (rightFly != null) rightFly.setPower(flyCmd);

                    if (frontIntake != null) frontIntake.setPower(0);
                    if (backIntake != null) backIntake.setPower(0);
                    if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                    if (System.currentTimeMillis() - stateTimer >= spinupRemainingMs) {
                        stateTimer = System.currentTimeMillis();
                        if (trigger != null) trigger.setPosition(TRIGGER_FIRE);
                        shotRetryCount = 0;
                        launchState = LaunchState.FIRE_1;
                    }
                    break;
                }

                case FIRE_1:
                    holdFireThenReset(LaunchState.RESET_AFTER_1, flyCmd);
                    break;

                case RESET_AFTER_1:
                    if (retryIfCenterStillHasBall(LaunchState.FIRE_1, flyCmd)) break;

                    stateTimer = System.currentTimeMillis();
                    if (planFrontShot) launchState = LaunchState.FEED_2;
                    else if (planBackShot) launchState = LaunchState.FEED_3;
                    else launchState = LaunchState.SPINDOWN;
                    break;

                case FEED_2:
                    if (leftFly != null) leftFly.setPower(flyCmd);
                    if (rightFly != null) rightFly.setPower(flyCmd);
                    if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                    if (frontIntake != null) frontIntake.setPower(intakeCommand(+FRONT_FEED_POWER, vbat));
                    if (backIntake != null) backIntake.setPower(0);

                    if (System.currentTimeMillis() - stateTimer >= FEED_TO_CENTER_MS) {
                        if (frontIntake != null) frontIntake.setPower(0);
                        if (backIntake != null) backIntake.setPower(0);

                        if (trigger != null) trigger.setPosition(TRIGGER_FIRE);
                        shotRetryCount = 0;
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.FIRE_2;
                    }
                    break;

                case FIRE_2:
                    holdFireThenReset(LaunchState.RESET_AFTER_2, flyCmd);
                    break;

                case RESET_AFTER_2:
                    if (retryIfCenterStillHasBall(LaunchState.FIRE_2, flyCmd)) break;

                    stateTimer = System.currentTimeMillis();
                    if (planBackShot) launchState = LaunchState.FEED_3;
                    else launchState = LaunchState.SPINDOWN;
                    break;

                case FEED_3:
                    if (leftFly != null) leftFly.setPower(flyCmd);
                    if (rightFly != null) rightFly.setPower(flyCmd);
                    if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                    if (backIntake != null) backIntake.setPower(intakeCommand(-BACK_FEED_POWER, vbat));
                    if (frontIntake != null) frontIntake.setPower(0);

                    if (System.currentTimeMillis() - stateTimer >= FEED_TO_CENTER_MS) {
                        if (backIntake != null) backIntake.setPower(0);
                        if (frontIntake != null) frontIntake.setPower(0);

                        if (trigger != null) trigger.setPosition(TRIGGER_FIRE);
                        shotRetryCount = 0;
                        stateTimer = System.currentTimeMillis();
                        launchState = LaunchState.FIRE_3;
                    }
                    break;

                case FIRE_3:
                    holdFireThenReset(LaunchState.RESET_AFTER_3, flyCmd);
                    break;

                case RESET_AFTER_3:
                    if (retryIfCenterStillHasBall(LaunchState.FIRE_3, flyCmd)) break;
                    stateTimer = System.currentTimeMillis();
                    launchState = LaunchState.SPINDOWN;
                    break;

                case SPINDOWN:
                    if (leftFly != null) leftFly.setPower(flyCmd);
                    if (rightFly != null) rightFly.setPower(flyCmd);

                    if (frontIntake != null) frontIntake.setPower(0);
                    if (backIntake != null) backIntake.setPower(0);
                    if (trigger != null) trigger.setPosition(TRIGGER_HOME);

                    if (System.currentTimeMillis() - stateTimer >= FLYWHEEL_SPINDOWN_MS) {
                        if (leftFly != null) leftFly.setPower(0);
                        if (rightFly != null) rightFly.setPower(0);
                        launchState = LaunchState.IDLE;

                        spinupRemainingMs = 0;
                        planCenterShot = false;
                        planFrontShot = false;
                        planBackShot = false;
                    }
                    break;
            }

            /* ===================== TELEMETRY ===================== */
            telemetry.addLine("BotHeading: " + imu.getRobotYawPitchRollAngles().getYaw());

            telemetryTag24(tagRangeIn, bearingDeg, centerOffsetDeg);
            telemetry.addData("LightPos", (light != null) ? "%.3f" : "null", (light != null) ? light.getPosition() : 0.0);

            telemetry.addData("TurretPos", "%.3f", currentTurretRotation);
            telemetry.addData("BallsPresent", "F:%s C:%s B:%s", frontHasBall, centerHasBall, backHasBall);

            // "Before launch" ball colors (only show while IDLE)
            if (launchState == LaunchState.IDLE) {
                telemetry.addData("BallColorFront",  classifyBallColor(frontColor));
                telemetry.addData("BallColorCenter", classifyBallColor(centerColor));
                telemetry.addData("BallColorBack",   classifyBallColor(backColor));
            }

            telemetry.addData("FullStackLatched", fullStackLatched);
            telemetry.addData("Plan", "C:%s F:%s B:%s", planCenterShot, planFrontShot, planBackShot);
            telemetry.addData("PreSpin", preSpinLatched);
            telemetry.addData("LaunchState", launchState);

            telemetry.update();
        }

        if (visionPortal != null) visionPortal.close();
    }

    /* ===================== LAUNCH HELPERS ===================== */
    private void holdFireThenReset(LaunchState nextResetState, double flyCmd) {
        if (leftFly != null) leftFly.setPower(flyCmd);
        if (rightFly != null) rightFly.setPower(flyCmd);

        if (frontIntake != null) frontIntake.setPower(0);
        if (backIntake != null) backIntake.setPower(0);

        if (System.currentTimeMillis() - stateTimer >= LAUNCH_TRIGGER_HOLD_MS) {
            if (trigger != null) trigger.setPosition(TRIGGER_HOME);
            stateTimer = System.currentTimeMillis();
            launchState = nextResetState;
        }
    }

    private boolean retryIfCenterStillHasBall(LaunchState fireStateToRetry, double flyCmd) {
        if (leftFly != null) leftFly.setPower(flyCmd);
        if (rightFly != null) rightFly.setPower(flyCmd);

        if (frontIntake != null) frontIntake.setPower(0);
        if (backIntake != null) backIntake.setPower(0);
        if (trigger != null) trigger.setPosition(TRIGGER_HOME);

        long elapsed = System.currentTimeMillis() - stateTimer;
        if (elapsed < TRIGGER_RESET_WAIT_MS) return true;

        if (centerHasBall) {
            if (elapsed < (TRIGGER_RESET_WAIT_MS + RETRY_EXTRA_WAIT_MS)) return true;

            if (shotRetryCount >= MAX_RETRIES_PER_SHOT) {
                abortLaunch();
                return true;
            }
            shotRetryCount++;
            if (trigger != null) trigger.setPosition(TRIGGER_FIRE);
            stateTimer = System.currentTimeMillis();
            launchState = fireStateToRetry;
            return true;
        }

        shotRetryCount = 0;
        return false;
    }

    /* ===================== VOLTAGE COMP HELPERS ===================== */
    private double flywheelCommand(double vbat) {
        if (Double.isNaN(vbat) || Double.isInfinite(vbat) || vbat <= 0.5) return multiplier;

        double base = (vbat < REF_VOLTAGE) ? 1.0 : (REF_VOLTAGE / vbat);
        return clamp(multiplier * base, 0.0, 1.0);
    }

    /* ===================== SMART INTAKE LOGIC ===================== */
    private void applySmartIntake(double vbat, double flyCmd) {
        DcMotor fastMotor, slowMotor;

        double dir = intakeModeOne ? +1.0 : -1.0;

        if (intakeModeOne) {
            fastMotor = frontIntake;
            slowMotor = backIntake;
        } else {
            fastMotor = backIntake;
            slowMotor = frontIntake;
        }

        boolean fastBall = intakeModeOne ? frontHasBall : backHasBall;
        boolean slowBall = intakeModeOne ? backHasBall : frontHasBall;

        double slowPower = slowBall ? 0.0 : (dir * MANUAL_SLOW);

        double fastPower;
        if (!slowBall) {
            fastPower = dir * MANUAL_FAST;
        } else {
            // 2 balls present (slow slot occupied): fast intake runs 7% slower than your current values
            double baseFast = (centerHasBall ? FAST_AFTER_SLOW_AND_CENTER : FAST_AFTER_SLOW);
            fastPower = dir * baseFast * TWO_BALL_SLOW_FACTOR;

            // AUTO STOP FULLY when center + fast slot are both occupied
            if (centerHasBall && fastBall) {
                fastPower = 0.0;
                slowPower = 0.0;
                isIntakeOn = false;

                fullStackLatched = true;

                if (AUTO_PRESPIN_WHEN_FULL && launchState == LaunchState.IDLE) {
                    preSpinLatched = true;
                    preSpinStartMs = System.currentTimeMillis();
                    if (leftFly != null) leftFly.setPower(flyCmd);
                    if (rightFly != null) rightFly.setPower(flyCmd);
                }
            }
        }

        double fastCmd = intakeCommand(fastPower, vbat);
        double slowCmd = intakeCommand(slowPower, vbat);

        if (fastMotor != null) fastMotor.setPower(fastCmd);
        if (slowMotor != null) slowMotor.setPower(slowCmd);
    }

    /* ===================== ABORT ===================== */
    private void abortLaunch() {
        if (frontIntake != null) frontIntake.setPower(0);
        if (backIntake != null) backIntake.setPower(0);
        if (leftFly != null) leftFly.setPower(0);
        if (rightFly != null) rightFly.setPower(0);
        if (trigger != null) trigger.setPosition(TRIGGER_HOME);

        shotRetryCount = 0;
        launchState = LaunchState.IDLE;

        planCenterShot = false;
        planFrontShot = false;
        planBackShot = false;

        preSpinLatched = false;
        preSpinStartMs = 0;
        spinupRemainingMs = 0;
        ltPrev = false;

        fullStackLatched = false;
    }

    /* ===================== DISTANCE HELPERS ===================== */
    private double safeDistanceCm(DistanceSensor s) {
        if (s == null) return 999.0;
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

        if (steer != null) steer.setPower(steerPower);
        if (drive != null) drive.setPower(speed);
    }

    private double getRawAngle(AnalogInput enc) {
        if (enc == null) return 0.0;
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
            if (m != null) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        if (turretRotation1 != null) turretRotation1.setDirection(Servo.Direction.FORWARD);
        if (turretRotation2 != null) turretRotation2.setDirection(Servo.Direction.REVERSE);

        trigger  = hardwareMap.get(Servo.class, "trigger");
        adjuster = hardwareMap.get(Servo.class, "adjuster");
        sideSort = hardwareMap.get(Servo.class, "side_sort");

        // config name is "lights"
        light = hardwareMap.get(Servo.class, "lights");

        try {
            RevColorSensorV3 f = hardwareMap.get(RevColorSensorV3.class, "frontColor");
            RevColorSensorV3 c = hardwareMap.get(RevColorSensorV3.class, "centerColor");
            RevColorSensorV3 b = hardwareMap.get(RevColorSensorV3.class, "backColor");

            frontColor  = f;
            centerColor = c;
            backColor   = b;

            frontDist  = f;
            centerDist = c;
            backDist   = b;

        } catch (Exception e) {
            frontColor  = hardwareMap.get(NormalizedColorSensor.class, "frontColor");
            centerColor = hardwareMap.get(NormalizedColorSensor.class, "centerColor");
            backColor   = hardwareMap.get(NormalizedColorSensor.class, "backColor");

            frontDist  = hardwareMap.get(DistanceSensor.class, "frontColor");
            centerDist = hardwareMap.get(DistanceSensor.class, "centerColor");
            backDist   = hardwareMap.get(DistanceSensor.class, "backColor");
        }
    }
}
