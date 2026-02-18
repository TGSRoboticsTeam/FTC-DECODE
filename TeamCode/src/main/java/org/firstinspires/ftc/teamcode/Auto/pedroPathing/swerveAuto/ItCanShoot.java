package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.RGB;
import org.firstinspires.ftc.teamcode.Tools.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.Tools.TurretMechanism;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@Autonomous(name = "ItCanShoot", group = "Testing")
public class ItCanShoot extends OpMode {
    private Follower follower;
    private FtcDashboard dashboard;
    private PanelsTelemetry pt;
    private TurretMechanism turret;

    // --- Mechanism Hardware ---
    private DcMotor leftFly, rightFly, frontIntake, backIntake;
    private Servo pusher, adjuster;

    // --- Ball sensors (config names: frontColor/centerColor/backColor) ---
    private DistanceSensor frontDist, centerDist, backDist;

    private AnalogInput flEnc, frEnc, blEnc, brEnc;
    private final double VOLTAGE_TO_RAD = (2 * Math.PI) / 3.3;

    public static double ROBOT_WIDTH = 18.0;
    public static double ROBOT_LENGTH = 18.0;

    // Trigger servo
    public static double TRIGGER_HOME = 0.225;
    public static double TRIGGER_FIRE = 0.0;

    // ===================== TELEOP-LAUNCH CONSTANTS (ORDER + TIMING) =====================
    // Outward burp (TeleOp)
    public static double SPINUP_OUTWARD_POWER = 0.4;
    public static long SPINUP_OUTWARD_MS = 40;

    // Launch feeding (TeleOp)
    public static double FRONT_FEED_POWER = 0.65;

    // TeleOp uses BACK_FEED_POWER = 0.375, then applies negative sign when feeding
    public static double BACK_FEED_POWER = 0.375;

    // Third shot feed slower (your request)
    public static double BACK_FEED_POWER_THIRD = BACK_FEED_POWER * 1; // tune (0.80..0.95)

    public static long FEED_TO_CENTER_MS = 450;

    // Trigger timing (TeleOp)
    public static long LAUNCH_TRIGGER_HOLD_MS = 300;
    public static long TRIGGER_RESET_WAIT_MS = 125;
    public static long RETRY_EXTRA_WAIT_MS = 350;

    // Spin timing (TeleOp)
    public static long FLYWHEEL_SPINUP_MS = 1000;
    public static long FLYWHEEL_SPINDOWN_MS = 300;

    // Retry control (TeleOp)
    public static int MAX_RETRIES_PER_SHOT = 3;

    // ===================== CENTER-ONLY BALL DETECTION (HYSTERESIS) =====================
    // Only CENTER sensor is used to decide whether a retry is needed.
    public static double CENTER_ON_CM = 4.0, CENTER_OFF_CM = 5.0;
    private boolean centerHasBall = false;
    // ================================================================================

    public Timer timer;
    private Servo lights;
    private AprilTagWebcam aprilTagWebcam;

    private int ballOrder = 21; // still set by tags in init_loop

    // shoot once
    private int shootState = 0;

    @Override
    public void init() {
        timer = new Timer();
        initMechanisms();

        turret = new TurretMechanism();
        turret.init(hardwareMap, telemetry, 24);

        aprilTagWebcam.init(hardwareMap, telemetry);

        dashboard = FtcDashboard.getInstance();
        pt = PanelsTelemetry.INSTANCE;
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        follower = SwerveConstants.createFollower(hardwareMap);

        flEnc = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frEnc = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        blEnc = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        brEnc = hardwareMap.get(AnalogInput.class, "backRightEncoder");

        lights = hardwareMap.get(Servo.class, "lights");

        if (pusher != null) pusher.setPosition(TRIGGER_HOME);

        telemetry.addData("Status", "Initialized (fireThree ALWAYS 3 shots; retry uses CENTER sensor only)");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        aprilTagWebcam.update();
        AprilTagDetection id23 = aprilTagWebcam.getTagBySpecificId(23);//purple
        AprilTagDetection id22 = aprilTagWebcam.getTagBySpecificId(22);//orange
        AprilTagDetection id21 = aprilTagWebcam.getTagBySpecificId(21);//green

        if (id21 != null) {
            telemetry.addLine("Tag 21 Detected");
            ballOrder = 21;
            lights.setPosition(RGB.green);
        }
        if (id22 != null) {
            telemetry.addLine("Tag 22 Detected");
            ballOrder = 22;
            lights.setPosition(RGB.orange);
        }
        if (id23 != null) {
            telemetry.addLine("Tag 23 Detected");
            ballOrder = 23;
            lights.setPosition(RGB.violet);
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        Pose currentPose = follower.getPose();

        if (shootState == 0) {
            turret.setServos(.65);

            // Flywheel command (auto style voltage comp)
            double flyCmd = getfarPower(0.875);
            setFlyHold(flyCmd);

            if (adjuster != null) adjuster.setPosition(0);

            fireThree(flyCmd);   // <<< ALWAYS fires 3 shots, retry uses CENTER sensor only

            lights.setPosition(RGB.green);
            shootState = 1;
        } else {
            stopIntakes();
            setFlyHold(0.0);
        }

        // telemetry
        double fle = flEnc.getVoltage() * VOLTAGE_TO_RAD;
        double fre = frEnc.getVoltage() * VOLTAGE_TO_RAD;
        double ble = blEnc.getVoltage() * VOLTAGE_TO_RAD;
        double bre = brEnc.getVoltage() * VOLTAGE_TO_RAD;

        updateCenterPresence();
        telemetry.addData("Shoot State", shootState);
        telemetry.addData("BallOrder(tag)", ballOrder);

        telemetry.addData("CenterHasBall(hyst)", centerHasBall);
        telemetry.addData("FrontDist(cm)", "%.2f", safeDistanceCm(frontDist));
        telemetry.addData("CenterDist(cm)", "%.2f", safeDistanceCm(centerDist));
        telemetry.addData("BackDist(cm)", "%.2f", safeDistanceCm(backDist));

        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(currentPose.getHeading()));

        telemetry.addData("FL_Rad", fle);
        telemetry.addData("FR_Rad", fre);
        telemetry.addData("BL_Rad", ble);
        telemetry.addData("BR_Rad", bre);

        drawToDashboard(currentPose);
        telemetry.update();
    }

    // =========================
    // fireThree: ALWAYS 3 shots
    // Retry decision: CENTER sensor only
    // =========================
    private void fireThree(double flyCmd) {
        telemetry.addLine("=== fireThree: ALWAYS 3 shots (center retry only) ===");
        telemetry.update();

        // Start with BURP then SPINUP (TeleOp order)
        if (pusher != null) pusher.setPosition(TRIGGER_HOME);
        setFlyHold(flyCmd);
        stopIntakes();

        // 1) BURP outward
        outwardBurp();

        // 2) SPINUP wait
        sleepMs(FLYWHEEL_SPINUP_MS);

        // 3) SHOT 1: FIRE center (retry uses center sensor only)
        fireOneWithCenterOnlyRetry(flyCmd);

        // 4) SHOT 2: FEED FRONT -> FIRE (retry uses center sensor only)
        feedFrontToCenter();
        fireOneWithCenterOnlyRetry(flyCmd);

        // 5) SHOT 3: FEED BACK (slower) -> FIRE (retry uses center sensor only)
        feedBackToCenterSlower();
        fireOneWithCenterOnlyRetry(flyCmd);

        // 6) SPINDOWN
        stopIntakes();
        if (pusher != null) pusher.setPosition(TRIGGER_HOME);
        sleepMs(FLYWHEEL_SPINDOWN_MS);
        setFlyHold(0.0);
    }

    /**
     * Matches TeleOp LaunchState.BURP behavior:
     * frontIntake = -SPINUP_OUTWARD_POWER
     * backIntake  = +SPINUP_OUTWARD_POWER
     * wait SPINUP_OUTWARD_MS then stop.
     */
    private void outwardBurp() {
        if (pusher != null) pusher.setPosition(TRIGGER_HOME);

        if (frontIntake != null) frontIntake.setPower(-SPINUP_OUTWARD_POWER);
        if (backIntake != null)  backIntake.setPower(+SPINUP_OUTWARD_POWER);

        sleepMs(SPINUP_OUTWARD_MS);
        stopIntakes();
    }

    /**
     * TeleOp-style retry loop, BUT ONLY checks CENTER sensor (hysteresis centerHasBall).
     */
    private void fireOneWithCenterOnlyRetry(double flyCmd) {
        int retries = 0;

        while (true) {
            setFlyHold(flyCmd);
            stopIntakes();

            // FIRE hold
            if (pusher != null) pusher.setPosition(TRIGGER_FIRE);
            sleepMs(LAUNCH_TRIGGER_HOLD_MS);

            // RESET wait
            if (pusher != null) pusher.setPosition(TRIGGER_HOME);
            sleepMs(TRIGGER_RESET_WAIT_MS);

            // Check center only
            updateCenterPresence();

            if (!centerHasBall) {
                // success
                return;
            }

            // still has ball -> extra wait then retry (TeleOp behavior)
            sleepMs(RETRY_EXTRA_WAIT_MS);

            retries++;
            if (retries >= MAX_RETRIES_PER_SHOT) {
                // abort this shot
                stopIntakes();
                setFlyHold(0.0);
                if (pusher != null) pusher.setPosition(TRIGGER_HOME);
                return;
            }
        }
    }

    private void feedFrontToCenter() {
        if (pusher != null) pusher.setPosition(TRIGGER_HOME);

        if (frontIntake != null) frontIntake.setPower(+FRONT_FEED_POWER);
        if (backIntake != null)  backIntake.setPower(0);

        sleepMs(FEED_TO_CENTER_MS);
        stopIntakes();
    }

    private void feedBackToCenterSlower() {
        if (pusher != null) pusher.setPosition(TRIGGER_HOME);

        // back feed direction is negative (TeleOp style)
        if (backIntake != null)  backIntake.setPower(-BACK_FEED_POWER_THIRD);
        if (frontIntake != null) frontIntake.setPower(0);

        sleepMs(FEED_TO_CENTER_MS);
        stopIntakes();
    }

    // =========================
    // CENTER-ONLY presence (hysteresis)
    // =========================
    private void updateCenterPresence() {
        double cCm = safeDistanceCm(centerDist);
        centerHasBall = hysteresisBall(centerHasBall, cCm, CENTER_ON_CM, CENTER_OFF_CM);
    }

    private double safeDistanceCm(DistanceSensor s) {
        if (s == null) return 999.0;
        double cm = s.getDistance(DistanceUnit.CM);
        if (Double.isNaN(cm) || Double.isInfinite(cm)) return 999.0;
        return cm;
    }

    private boolean hysteresisBall(boolean prev, double cm, double onCm, double offCm) {
        return prev ? (cm <= offCm) : (cm <= onCm);
    }

    // =========================
    // Existing helpers
    // =========================
    public double getfarPower(double p) {
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        return p * 12.0 / voltage;
    }

    private void setFlyHold(double power) {
        // Keep your auto convention: leftFly negative, rightFly positive
        if (leftFly != null) leftFly.setPower(-power);
        if (rightFly != null) rightFly.setPower(+power);
    }

    private void stopIntakes() {
        if (frontIntake != null) frontIntake.setPower(0);
        if (backIntake != null) backIntake.setPower(0);
    }

    private void sleepMs(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private void drawToDashboard(Pose currentPose) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStroke("#3F51B5");
        fieldOverlay.strokeRect(
                currentPose.getX() - (ROBOT_LENGTH / 2),
                currentPose.getY() - (ROBOT_WIDTH / 2),
                ROBOT_LENGTH,
                ROBOT_WIDTH
        );

        dashboard.sendTelemetryPacket(packet);
    }

    private void initMechanisms() {
        leftFly = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        pusher = hardwareMap.get(Servo.class, "trigger");
        adjuster = hardwareMap.get(Servo.class, "adjuster");

        aprilTagWebcam = new AprilTagWebcam();

        // Your sensor names:
        frontDist  = hardwareMap.get(DistanceSensor.class, "frontColor");
        centerDist = hardwareMap.get(DistanceSensor.class, "centerColor");
        backDist   = hardwareMap.get(DistanceSensor.class, "backColor");
    }
}
