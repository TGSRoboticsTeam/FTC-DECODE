package org.firstinspires.ftc.teamcode.Auto;

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
    private GoBildaPinpointDriver odo; // RC config name: "odo" (per your sample)

    // Your SDK doesn't support EncoderDirection.REVERSE, so invert in SOFTWARE:
    private static final double PINPOINT_X_SIGN = -1.0; // invert X pod reading
    // Pinpoint Y+ is LEFT; we want +Y = RIGHT in our functions:
    private static final double PINPOINT_Y_RIGHT_SIGN = -1.0;

    /* ===================== SWERVE HARDWARE ===================== */
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;

    /* ===================== STEERING OFFSETS (your TeleOp values) ===================== */
    final double FRONT_LEFT_OFFSET  = 1.34;
    final double FRONT_RIGHT_OFFSET = 3.161;
    final double BACK_LEFT_OFFSET   = 1.589;
    final double BACK_RIGHT_OFFSET  = 1.237;

    /* ===================== STEER CONTROL ===================== */
    final double STEER_KP = 0.7;
    final double STEER_DEADBAND = 0.05;

    private static final double ALIGN_TOL_RAD = Math.toRadians(8.0);
    private static final long   ALIGN_TIMEOUT_MS = 900;

    /* ===================== DRIVE CONTROL ===================== */
    // X distance PID (P + small D)
    private final PID xPID = new PID(0.035, 0.0, 0.004);
    // Y distance PID (P + small D)
    private final PID yPID = new PID(0.035, 0.0, 0.004);

    // Heading hold used as a correction term
    private final PID hPID = new PID(1.6, 0.0, 0.08);

    private static final double MAX_TRANSLATE = 0.55;
    private static final double MAX_TURN = 0.20;

    private static final double POS_TOL_IN = 0.75;
    private static final double HEAD_TOL_RAD = Math.toRadians(2.0);

    private static final double POWER_RAMP_PER_SEC = 1.8;

    @Override
    public void runOpMode() {
        initHardware();
        initOdometry();

        telemetry.addLine("GPTAutoDrive ready");
        telemetry.addLine("X-only + Y-only distance functions, with module pre-align");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Demo (replace)
        driveX(24);
        sleep(250);
        driveYRight(18);
        sleep(250);
        driveX(-12);
        sleep(250);
        driveYRight(-18);

        stopAll();
    }

    /* ===================== PINPOINT INIT (your config) ===================== */
    private void initOdometry() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // NOTE: This SDK version doesn't have REVERSE, so keep FORWARD/FORWARD.
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Must be still here
        odo.resetPosAndIMU();
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        odo.update();
    }

    /* ===================== PUBLIC: DRIVE STRAIGHT IN X ===================== */
    // +in = forward, -in = backward
    public void driveX(double inches) {
        Pose2D start = readPose();
        double startX = xIn(start);
        double targetX = startX + inches;
        double holdHeading = headingRad(start);

        xPID.reset();
        hPID.reset();

        // Point all modules straight forward/back BEFORE driving
        double desiredAngle = (inches >= 0) ? 0.0 : Math.PI; // 0 rad = forward, pi = backward
        alignAllModulesTo(desiredAngle);

        ElapsedTime loopT = new ElapsedTime();
        loopT.reset();

        double baseSmoothed = 0.0;

        while (opModeIsActive()) {
            Pose2D p = readPose();
            double dt = Math.max(0.01, loopT.seconds());
            loopT.reset();

            double xErr = targetX - xIn(p);
            double hErr = wrapRad(holdHeading - headingRad(p));

            if (Math.abs(xErr) <= POS_TOL_IN && Math.abs(hErr) <= HEAD_TOL_RAD) break;

            double base = xPID.update(xErr, dt);
            base = clamp(base, -MAX_TRANSLATE, MAX_TRANSLATE);
            baseSmoothed = ramp(baseSmoothed, base, POWER_RAMP_PER_SEC, dt);

            double turn = hPID.update(hErr, dt);
            turn = clamp(turn, -MAX_TURN, MAX_TURN);

            // Heading correction: left/right differential
            double leftPower  = clamp(baseSmoothed - turn, -1.0, 1.0);
            double rightPower = clamp(baseSmoothed + turn, -1.0, 1.0);

            // Keep modules locked while moving
            steerHold(desiredAngle);

            frontLeftDrive.setPower(leftPower);
            backLeftDrive.setPower(leftPower);
            frontRightDrive.setPower(rightPower);
            backRightDrive.setPower(rightPower);

            telemetry.addLine("driveX()");
            telemetry.addData("X(in)", "%.2f", xIn(p));
            telemetry.addData("TargetX(in)", "%.2f", targetX);
            telemetry.addData("Xerr(in)", "%.2f", xErr);
            telemetry.addData("Head(deg)", "%.1f", p.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Herr(deg)", "%.2f", Math.toDegrees(hErr));
            telemetry.addData("Base", "%.2f", baseSmoothed);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addData("L/R", "%.2f / %.2f", leftPower, rightPower);
            telemetry.update();
        }

        stopAll();
        sleep(60);
    }

    /* ===================== PUBLIC: DRIVE STRAIGHT IN Y ONLY ===================== */
    // +in = right, -in = left
    public void driveYRight(double inchesRight) {
        Pose2D start = readPose();
        double startY = yRightIn(start);
        double targetY = startY + inchesRight;
        double holdHeading = headingRad(start);

        yPID.reset();
        hPID.reset();

        // For strafe right: wheels point +90deg. For strafe left: -90deg (or 270)
        double desiredAngle = (inchesRight >= 0) ? (Math.PI / 2.0) : (-Math.PI / 2.0);
        alignAllModulesTo(desiredAngle);

        ElapsedTime loopT = new ElapsedTime();
        loopT.reset();

        double baseSmoothed = 0.0;

        while (opModeIsActive()) {
            Pose2D p = readPose();
            double dt = Math.max(0.01, loopT.seconds());
            loopT.reset();

            double yErr = targetY - yRightIn(p);
            double hErr = wrapRad(holdHeading - headingRad(p));

            if (Math.abs(yErr) <= POS_TOL_IN && Math.abs(hErr) <= HEAD_TOL_RAD) break;

            double base = yPID.update(yErr, dt);
            base = clamp(base, -MAX_TRANSLATE, MAX_TRANSLATE);
            baseSmoothed = ramp(baseSmoothed, base, POWER_RAMP_PER_SEC, dt);

            double turn = hPID.update(hErr, dt);
            turn = clamp(turn, -MAX_TURN, MAX_TURN);

            // For pure strafe with heading correction:
            // Add yaw via diagonals so you don’t “tank steer” while wheels are sideways.
            double fl = clamp(baseSmoothed + turn, -1.0, 1.0);
            double br = clamp(baseSmoothed + turn, -1.0, 1.0);
            double fr = clamp(baseSmoothed - turn, -1.0, 1.0);
            double bl = clamp(baseSmoothed - turn, -1.0, 1.0);

            steerHold(desiredAngle);

            frontLeftDrive.setPower(fl);
            backRightDrive.setPower(br);
            frontRightDrive.setPower(fr);
            backLeftDrive.setPower(bl);

            telemetry.addLine("driveYRight()");
            telemetry.addData("Y_R(in)", "%.2f", yRightIn(p));
            telemetry.addData("TargetY_R(in)", "%.2f", targetY);
            telemetry.addData("Yerr(in)", "%.2f", yErr);
            telemetry.addData("Head(deg)", "%.1f", p.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Herr(deg)", "%.2f", Math.toDegrees(hErr));
            telemetry.addData("Base", "%.2f", baseSmoothed);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.update();
        }

        stopAll();
        sleep(60);
    }

    /* ===================== MODULE ALIGN / HOLD ===================== */
    private void alignAllModulesTo(double targetAngleRad) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start) < ALIGN_TIMEOUT_MS) {
            runSteer(frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  targetAngleRad);
            runSteer(frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, targetAngleRad);
            runSteer(backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   targetAngleRad);
            runSteer(backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  targetAngleRad);

            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            if (allAligned(targetAngleRad, ALIGN_TOL_RAD)) break;
        }

        frontLeftSteer.setPower(0);
        frontRightSteer.setPower(0);
        backLeftSteer.setPower(0);
        backRightSteer.setPower(0);
        sleep(40);
    }

    private void steerHold(double targetAngleRad) {
        runSteer(frontLeftSteer,  frontLeftEncoder,  FRONT_LEFT_OFFSET,  targetAngleRad);
        runSteer(frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, targetAngleRad);
        runSteer(backLeftSteer,   backLeftEncoder,   BACK_LEFT_OFFSET,   targetAngleRad);
        runSteer(backRightSteer,  backRightEncoder,  BACK_RIGHT_OFFSET,  targetAngleRad);
    }

    private boolean allAligned(double targetAngleRad, double tolRad) {
        double eFL = Math.abs(wrapAngle(targetAngleRad - moduleAngle(frontLeftEncoder, FRONT_LEFT_OFFSET)));
        double eFR = Math.abs(wrapAngle(targetAngleRad - moduleAngle(frontRightEncoder, FRONT_RIGHT_OFFSET)));
        double eBL = Math.abs(wrapAngle(targetAngleRad - moduleAngle(backLeftEncoder, BACK_LEFT_OFFSET)));
        double eBR = Math.abs(wrapAngle(targetAngleRad - moduleAngle(backRightEncoder, BACK_RIGHT_OFFSET)));
        return (eFL < tolRad && eFR < tolRad && eBL < tolRad && eBR < tolRad);
    }

    private void runSteer(CRServo steer, AnalogInput enc, double offset, double targetAng) {
        double current = moduleAngle(enc, offset);
        double delta = wrapAngle(targetAng - current);

        if (Math.abs(delta) > Math.PI / 2) delta = wrapAngle(delta + Math.PI);

        double power = clamp(-STEER_KP * delta, -1, 1);
        if (Math.abs(power) < STEER_DEADBAND) power = 0;
        steer.setPower(power);
    }

    private double moduleAngle(AnalogInput enc, double offset) {
        return wrapAngle(getRawAngle(enc) - offset);
    }

    /* ===================== POSE HELPERS ===================== */
    private Pose2D readPose() {
        odo.update();
        return odo.getPosition();
    }

    private double xIn(Pose2D p) {
        return PINPOINT_X_SIGN * p.getX(DistanceUnit.INCH);
    }

    private double yRightIn(Pose2D p) {
        return PINPOINT_Y_RIGHT_SIGN * p.getY(DistanceUnit.INCH);
    }

    private double headingRad(Pose2D p) {
        return Math.toRadians(p.getHeading(AngleUnit.DEGREES));
    }

    /* ===================== STOP ===================== */
    private void stopAll() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        frontLeftSteer.setPower(0);
        frontRightSteer.setPower(0);
        backLeftSteer.setPower(0);
        backRightSteer.setPower(0);
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

        // Match your TeleOp motor directions
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
