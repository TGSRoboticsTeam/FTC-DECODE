package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name = "GPTAutoDrive", group = "Auto")
public class GPTAutoDrive extends LinearOpMode {

    /* ===================== PINPOINT ===================== */
    private GoBildaPinpointDriver odo; // config name: "odo" (per your file)

    // Pinpoint reports: +Y = LEFT. You want: +Y = RIGHT => flip sign.
    private static final double PINPOINT_TO_Y_RIGHT_SIGN = -1.0;

    /* ===================== SWERVE HARDWARE ===================== */
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;

    /* ===================== SWERVE CONSTANTS (from your TeleOp) ===================== */
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    final double FRONT_LEFT_OFFSET  = 1.34;
    final double FRONT_RIGHT_OFFSET = 3.161;
    final double BACK_LEFT_OFFSET   = 1.589;
    final double BACK_RIGHT_OFFSET  = 1.237;

    // steering loop (same idea as TeleOp)
    final double STEER_KP = 0.6;
    final double STEER_DEADBAND = 0.05;

    /* ===================== CONTROL SETTINGS ===================== */
    // Distance control (keep this simple + stable): P + small D, no I
    private final PID xPID = new PID(0.035, 0.0, 0.004);
    private final PID yPID = new PID(0.035, 0.0, 0.004);

    // Heading hold (P + D), no I
    private final PID hPID = new PID(2.0, 0.0, 0.10);

    private static final double MAX_TRANSLATE = 0.45; // lower = smoother/straighter
    private static final double MAX_ROTATE    = 0.25;

    private static final double POS_TOL_IN = 0.75;
    private static final double HEAD_TOL_RAD = Math.toRadians(2.0);

    // Pre-align requirements
    private static final double ALIGN_TOL_RAD = Math.toRadians(10.0); // wheels considered "aimed"
    private static final long   ALIGN_TIMEOUT_MS = 700;              // don’t get stuck forever

    // gentle ramp to reduce snap/jitter
    private static final double POWER_RAMP_PER_SEC = 1.6; // max change in power per second

    @Override
    public void runOpMode() {
        initHardware();
        initOdometryLikeYourConfig();

        telemetry.addLine("GPTAutoDrive ready (pre-align wheels before driving).");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Demo: forward 24", right 12", back 24", left 12"
        driveX(24);
        driveY(12);
        driveX(-24);
        driveY(-12);

        stopModules();
    }

    /* ===================== YOUR PINPOINT CONFIG (mirrors your file) ===================== */
    private void initOdometryLikeYourConfig() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Must be still here
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        odo.update();
    }

    /* ===================== DRIVE-BY-DISTANCE (PUBLIC API) ===================== */
    // + forward, - back
    public void driveX(double inches) {
        Pose2D start = readPose();
        double startX = start.getX(DistanceUnit.INCH);
        double targetX = startX + inches;
        double holdHeading = headingRad(start);

        xPID.reset();
        hPID.reset();

        // Desired motion: forward/back only (robotY), no strafe
        // Step 1: pre-align wheels to that direction (point wheels BEFORE applying drive)
        preAlignForMotion(+1.0 * Math.signum(inches), 0.0, holdHeading);

        ElapsedTime loopT = new ElapsedTime();
        loopT.reset();
        double fwdCmdSmoothed = 0.0;

        while (opModeIsActive()) {
            Pose2D p = readPose();
            double dt = Math.max(0.01, loopT.seconds());
            loopT.reset();

            double xErr = targetX - p.getX(DistanceUnit.INCH);
            double hErr = wrapRad(holdHeading - headingRad(p));

            if (Math.abs(xErr) <= POS_TOL_IN && Math.abs(hErr) <= HEAD_TOL_RAD) break;

            // PID distance -> forward command
            double fwdCmd = xPID.update(xErr, dt);
            fwdCmd = clamp(fwdCmd, -MAX_TRANSLATE, MAX_TRANSLATE);

            // Smooth ramp to avoid jerk (helps straightness)
            fwdCmdSmoothed = ramp(fwdCmdSmoothed, fwdCmd, POWER_RAMP_PER_SEC, dt);

            // Heading correction
            double rotCmd = hPID.update(hErr, dt);
            rotCmd = clamp(rotCmd, -MAX_ROTATE, MAX_ROTATE);

            // Step 2: drive using already-aligned direction
            setSwerveDrive(fwdCmdSmoothed, 0.0, rotCmd);

            telemetry.addLine("driveX()");
            telemetry.addData("TargetX", "%.2f", targetX);
            telemetry.addData("X", "%.2f", p.getX(DistanceUnit.INCH));
            telemetry.addData("Xerr", "%.2f", xErr);
            telemetry.addData("Head(deg)", "%.1f", p.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Cmd fwd", "%.2f", fwdCmdSmoothed);
            telemetry.addData("Cmd rot", "%.2f", rotCmd);
            telemetry.update();
        }

        setSwerveDrive(0, 0, 0);
        sleep(60);
    }

    // + right, - left (your requested convention)
    public void driveY(double inchesRight) {
        Pose2D start = readPose();
        double startYRight = yRightIn(start);
        double targetYRight = startYRight + inchesRight;
        double holdHeading = headingRad(start);

        yPID.reset();
        hPID.reset();

        // Desired motion: strafe only (robotX)
        preAlignForMotion(0.0, +1.0 * Math.signum(inchesRight), holdHeading);

        ElapsedTime loopT = new ElapsedTime();
        loopT.reset();
        double strCmdSmoothed = 0.0;

        while (opModeIsActive()) {
            Pose2D p = readPose();
            double dt = Math.max(0.01, loopT.seconds());
            loopT.reset();

            double yErr = targetYRight - yRightIn(p);
            double hErr = wrapRad(holdHeading - headingRad(p));

            if (Math.abs(yErr) <= POS_TOL_IN && Math.abs(hErr) <= HEAD_TOL_RAD) break;

            double strCmd = yPID.update(yErr, dt);
            strCmd = clamp(strCmd, -MAX_TRANSLATE, MAX_TRANSLATE);

            strCmdSmoothed = ramp(strCmdSmoothed, strCmd, POWER_RAMP_PER_SEC, dt);

            double rotCmd = hPID.update(hErr, dt);
            rotCmd = clamp(rotCmd, -MAX_ROTATE, MAX_ROTATE);

            setSwerveDrive(0.0, strCmdSmoothed, rotCmd);

            telemetry.addLine("driveY()");
            telemetry.addData("TargetY_R", "%.2f", targetYRight);
            telemetry.addData("Y_R", "%.2f", yRightIn(p));
            telemetry.addData("Yerr", "%.2f", yErr);
            telemetry.addData("Head(deg)", "%.1f", p.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Cmd str", "%.2f", strCmdSmoothed);
            telemetry.addData("Cmd rot", "%.2f", rotCmd);
            telemetry.update();
        }

        setSwerveDrive(0, 0, 0);
        sleep(60);
    }

    /* ===================== PRE-ALIGN: SET WHEEL DIRECTION FIRST ===================== */
    // desiredRobotY: sign(+/-) or 0 ; desiredRobotX: sign(+/-) or 0 ; rot=0 during align
    private void preAlignForMotion(double desiredRobotY, double desiredRobotX, double holdHeadingRad) {
        // If both are zero, nothing to align
        if (Math.abs(desiredRobotY) < 1e-6 && Math.abs(desiredRobotX) < 1e-6) return;

        // Build a “direction-only” command: normalize to 1 so targets are stable
        double mag = Math.hypot(desiredRobotX, desiredRobotY);
        double robotX = desiredRobotX / mag;
        double robotY = desiredRobotY / mag;

        // Compute module targets for that direction with rot = 0
        ModuleTargets targets = computeTargets(robotY, robotX, 0.0);

        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < ALIGN_TIMEOUT_MS) {
            // Apply ONLY steering toward the targets; drive power = 0
            steerOnlyToTargets(targets);

            if (modulesAligned(targets, ALIGN_TOL_RAD)) break;

            telemetry.addLine("Aligning modules...");
            telemetry.update();
        }

        // Stop steering briefly to reduce “hunting” right before drive starts
        frontLeftSteer.setPower(0);
        frontRightSteer.setPower(0);
        backLeftSteer.setPower(0);
        backRightSteer.setPower(0);
        sleep(40);
    }

    private void steerOnlyToTargets(ModuleTargets t) {
        runSteer(frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  t.angFL);
        runSteer(frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, t.angFR);
        runSteer(backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   t.angBL);
        runSteer(backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  t.angBR);

        // Ensure drives are OFF during alignment
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    private boolean modulesAligned(ModuleTargets t, double tolRad) {
        double eFL = Math.abs(wrapAngle(t.angFL - currentModuleAngle(frontLeftEncoder, FRONT_LEFT_OFFSET)));
        double eFR = Math.abs(wrapAngle(t.angFR - currentModuleAngle(frontRightEncoder, FRONT_RIGHT_OFFSET)));
        double eBL = Math.abs(wrapAngle(t.angBL - currentModuleAngle(backLeftEncoder, BACK_LEFT_OFFSET)));
        double eBR = Math.abs(wrapAngle(t.angBR - currentModuleAngle(backRightEncoder, BACK_RIGHT_OFFSET)));
        return (eFL < tolRad && eFR < tolRad && eBL < tolRad && eBR < tolRad);
    }

    private void runSteer(CRServo steer, AnalogInput enc, double offset, double targetAng) {
        double current = currentModuleAngle(enc, offset);
        double delta = wrapAngle(targetAng - current);

        // same flip logic so steering goes shortest way
        if (Math.abs(delta) > Math.PI / 2) delta = wrapAngle(delta + Math.PI);

        double steerPower = clamp(-STEER_KP * delta, -1, 1);
        if (Math.abs(steerPower) < STEER_DEADBAND) steerPower = 0;
        steer.setPower(steerPower);
    }

    private double currentModuleAngle(AnalogInput enc, double offset) {
        return wrapAngle(getRawAngle(enc) - offset);
    }

    /* ===================== SWERVE DRIVE (same as your TeleOp structure) ===================== */
    private void setSwerveDrive(double robotY, double robotX, double rot) {
        ModuleTargets t = computeTargets(robotY, robotX, rot);

        runModule(frontLeftDrive,  frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  t.spdFL, t.angFL);
        runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, t.spdFR, t.angFR);
        runModule(backLeftDrive,   backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   t.spdBL, t.angBL);
        runModule(backRightDrive,  backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  t.spdBR, t.angBR);
    }

    private ModuleTargets computeTargets(double robotY, double robotX, double rot) {
        double A = robotX - rot * (WHEELBASE / R);
        double B = robotX + rot * (WHEELBASE / R);
        double C = robotY - rot * (TRACK_WIDTH / R);
        double D = robotY + rot * (TRACK_WIDTH / R);

        double spdFL = Math.hypot(B, D);
        double spdFR = Math.hypot(B, C);
        double spdBL = Math.hypot(A, D);
        double spdBR = Math.hypot(A, C);

        double max = Math.max(Math.max(spdFL, spdFR), Math.max(spdBL, spdBR));
        if (max > 1.0) {
            spdFL /= max; spdFR /= max; spdBL /= max; spdBR /= max;
        }

        double angFL = Math.atan2(B, D);
        double angFR = Math.atan2(B, C);
        double angBL = Math.atan2(A, D);
        double angBR = Math.atan2(A, C);

        return new ModuleTargets(spdFL, spdFR, spdBL, spdBR, angFL, angFR, angBL, angBR);
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

    private void stopModules() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        frontLeftSteer.setPower(0);
        frontRightSteer.setPower(0);
        backLeftSteer.setPower(0);
        backRightSteer.setPower(0);
    }

    /* ===================== POSE HELPERS ===================== */
    private Pose2D readPose() {
        odo.update();
        return odo.getPosition();
    }

    private double yRightIn(Pose2D p) {
        return PINPOINT_TO_Y_RIGHT_SIGN * p.getY(DistanceUnit.INCH);
    }

    private double headingRad(Pose2D p) {
        return Math.toRadians(p.getHeading(AngleUnit.DEGREES));
    }

    /* ===================== MATH / UTIL ===================== */
    private double getRawAngle(AnalogInput enc) {
        return enc.getVoltage() / 3.3 * (2 * Math.PI);
    }

    private double wrapAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private double wrapRad(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double ramp(double current, double target, double maxDeltaPerSec, double dt) {
        double maxDelta = maxDeltaPerSec * dt;
        double delta = target - current;
        if (delta > maxDelta) delta = maxDelta;
        if (delta < -maxDelta) delta = -maxDelta;
        return current + delta;
    }

    /* ===================== SMALL STRUCT ===================== */
    private static class ModuleTargets {
        final double spdFL, spdFR, spdBL, spdBR;
        final double angFL, angFR, angBL, angBR;
        ModuleTargets(double sfl, double sfr, double sbl, double sbr,
                      double afl, double afr, double abl, double abr) {
            spdFL = sfl; spdFR = sfr; spdBL = sbl; spdBR = sbr;
            angFL = afl; angFR = afr; angBL = abl; angBR = abr;
        }
    }

    /* ===================== PID ===================== */
    private static class PID {
        private final double kP, kI, kD;
        private double i = 0.0;
        private double prev = 0.0;
        private boolean hasPrev = false;

        PID(double p, double i, double d) { kP = p; kI = i; kD = d; }

        void reset() { this.i = 0.0; this.prev = 0.0; this.hasPrev = false; }

        double update(double err, double dt) {
            i += err * dt;
            double d = 0.0;
            if (hasPrev) d = (err - prev) / dt;
            prev = err;
            hasPrev = true;
            return kP * err + kI * i + kD * d;
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

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        // Match your TeleOp directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
