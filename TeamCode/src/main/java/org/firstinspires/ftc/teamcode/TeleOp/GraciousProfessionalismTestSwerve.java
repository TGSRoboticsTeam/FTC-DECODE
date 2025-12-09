package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field–centric swerve drive TeleOp for 4 Axon pods:
 *  - motorA/B/C/D : drive motors (GoBILDA 5000 series)
 *  - servoA/B/C/D : Axon Max+ MK1 in continuous mode (steering)
 *  - podA/B/C/D   : analog absolute encoders for pod angle
 *
 * Layout (top view, starting top-left and going clockwise):
 *      A (front-left)   B (front-right)
 *      D (back-left)    C (back-right)
 */
@TeleOp(name = "Swerve Field-Centric TeleOp", group = "TeleOp")
public class GraciousProfessionalismTestSwerve extends LinearOpMode { // G.P.T.Swerve, You're welcome.

    // --- Robot geometry (inches) ---
    // Use your drawing dimensions: width 17.258, length 13.544
    private static final double TRACK_WIDTH_IN  = 17.258;  // left-right distance between module centers
    private static final double WHEELBASE_IN    = 13.544;  // front-back distance between module centers

    // --- Steering PID (for Axon CR servo) ---
    private static final double STEER_KP = 1.2;   // start here, tune on robot
    private static final double STEER_KI = 0.0;
    private static final double STEER_KD = 0.02;

    // --- Heading auto-align ---
    private static final double HEADING_ALIGN_KP = 2.0; // for aligning to +X (0 rad)

    // --- Axon encoder voltage range ---
    private static final double ENCODER_MAX_VOLTAGE = 3.3; // or 5.0 depending on wiring

    // Offsets so that "zero" pod angle (wheels facing +X) corresponds to 0 rad
    // You will set/tune these using your pod-zero program.
    private static final double OFFSET_A_RAD = 0.0;
    private static final double OFFSET_B_RAD = 0.0;
    private static final double OFFSET_C_RAD = 0.0;
    private static final double OFFSET_D_RAD = 0.0;

    // Hardware
    private IMU imu;

    private DcMotor motorA, motorB, motorC, motorD;
    private CRServo servoA, servoB, servoC, servoD;
    private AnalogInput podA, podB, podC, podD;

    private SwerveModule moduleA, moduleB, moduleC, moduleD;

    // IMU heading offset (for zero-heading button)
    private double headingOffsetRad = 0.0;

    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // ---------------- IMU SETUP ----------------
        imu = hardwareMap.get(IMU.class, "imu");

        // If your Control Hub is mounted in a non-standard orientation,
        // configure it here. Example assumes logo UP, USB FORWARD:
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParams);

        // ---------------- HARDWARE MAP ----------------
        motorA = hardwareMap.get(DcMotor.class, "motorA"); // front-left
        motorB = hardwareMap.get(DcMotor.class, "motorB"); // front-right
        motorC = hardwareMap.get(DcMotor.class, "motorC"); // back-right
        motorD = hardwareMap.get(DcMotor.class, "motorD"); // back-left

        servoA = hardwareMap.get(CRServo.class, "servoA");
        servoB = hardwareMap.get(CRServo.class, "servoB");
        servoC = hardwareMap.get(CRServo.class, "servoC");
        servoD = hardwareMap.get(CRServo.class, "servoD");

        podA = hardwareMap.get(AnalogInput.class, "podA");
        podB = hardwareMap.get(AnalogInput.class, "podB");
        podC = hardwareMap.get(AnalogInput.class, "podC");
        podD = hardwareMap.get(AnalogInput.class, "podD");

        // Optional: set any motor directions here if needed
        motorA.setDirection(DcMotor.Direction.FORWARD);
        motorB.setDirection(DcMotor.Direction.REVERSE); // many drivetrains reverse right side
        motorC.setDirection(DcMotor.Direction.REVERSE);
        motorD.setDirection(DcMotor.Direction.FORWARD);

        motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---------------- SWERVE MODULE OBJECTS ----------------
        double R = Math.hypot(TRACK_WIDTH_IN, WHEELBASE_IN);

        // Front-Left (A)
        moduleA = new SwerveModule(
                motorA, servoA, podA,
                OFFSET_A_RAD,
                +WHEELBASE_IN / R, +TRACK_WIDTH_IN / R
        );
        // Front-Right (B)
        moduleB = new SwerveModule(
                motorB, servoB, podB,
                OFFSET_B_RAD,
                +WHEELBASE_IN / R, -TRACK_WIDTH_IN / R
        );
        // Back-Right (C)
        moduleC = new SwerveModule(
                motorC, servoC, podC,
                OFFSET_C_RAD,
                -WHEELBASE_IN / R, -TRACK_WIDTH_IN / R
        );
        // Back-Left (D)
        moduleD = new SwerveModule(
                motorD, servoD, podD,
                OFFSET_D_RAD,
                -WHEELBASE_IN / R, +TRACK_WIDTH_IN / R
        );

        telemetry.addLine("Swerve Field-Centric TeleOp ready");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        boolean lastY = false; // for edge-detect on zero-heading
        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            // ---------------- READ GAMEPAD ----------------
            double lx = gamepad1.left_stick_x;   // + right
            double ly = -gamepad1.left_stick_y;  // + forward (invert)
            double rx = gamepad1.right_stick_x;  // rotation

            // deadzone
            if (Math.abs(lx) < 0.02) lx = 0;
            if (Math.abs(ly) < 0.02) ly = 0;
            if (Math.abs(rx) < 0.02) rx = 0;

            // Zero heading (Y button, edge-detected)
            if (gamepad1.y && !lastY) {
                double rawHeading = getRawHeadingRad();
                headingOffsetRad = rawHeading; // so current becomes 0
            }
            lastY = gamepad1.y;

            // Current field-relative heading
            double robotHeading = getFieldHeadingRad();

            // Auto-align to +X (0 rad) while left-stick button held
            if (gamepad1.left_stick_button) {
                double error = wrapAngle(0.0 - robotHeading);
                rx = HEADING_ALIGN_KP * error;
                rx = clamp(rx, -1.0, 1.0);
            }

            // ---------------- FIELD-CENTRIC TRANSFORM ----------------
            // Convert joystick (field frame) to robot frame
            double cosH = Math.cos(robotHeading);
            double sinH = Math.sin(robotHeading);

            // Robot-relative velocities
            double vx = lx * cosH + ly * sinH;
            double vy = -lx * sinH + ly * cosH;
            double omega = rx; // rotational command

            // ---------------- APPLY TO SWERVE MODULES ----------------
            // Each module computes its own desired speed/angle given vx, vy, omega
            double maxSpeed = 0.0;

            maxSpeed = Math.max(maxSpeed, moduleA.computeDesiredState(vx, vy, omega));
            maxSpeed = Math.max(maxSpeed, moduleB.computeDesiredState(vx, vy, omega));
            maxSpeed = Math.max(maxSpeed, moduleC.computeDesiredState(vx, vy, omega));
            maxSpeed = Math.max(maxSpeed, moduleD.computeDesiredState(vx, vy, omega));

            // Normalize speeds so none exceed 1.0
            if (maxSpeed < 1e-3) maxSpeed = 1.0; // prevent divide-by-zero when stopped

            moduleA.apply(dt, maxSpeed);
            moduleB.apply(dt, maxSpeed);
            moduleC.apply(dt, maxSpeed);
            moduleD.apply(dt, maxSpeed);

            // ---------------- TELEMETRY ----------------
            telemetry.addData("Heading (deg)", Math.toDegrees(robotHeading));
            telemetry.addData("LX", lx);
            telemetry.addData("LY", ly);
            telemetry.addData("RX/omega", omega);
            telemetry.addLine("Y = zero heading, LS button = auto-align to +X");
            telemetry.update();
        }
    }

    // ========== HELPER METHODS ==========

    /** Raw yaw angle from IMU, radians, increasing CCW. */
    private double getRawHeadingRad() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** Field heading (with zero offset applied), radians in [-pi, pi]. */
    private double getFieldHeadingRad() {
        double angle = getRawHeadingRad() - headingOffsetRad;
        return wrapAngle(angle);
    }

    private static double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    // ========== INNER CLASS: SWERVE MODULE ==========

    /**
     * Represents one swerve module: drive motor, CR steering servo, analog encoder.
     * Uses simple PID on steering and module optimization (flip 180° if shorter).
     */
    private class SwerveModule {
        private final DcMotor drive;
        private final CRServo steer;
        private final AnalogInput encoder;
        private final double encoderOffset; // radians
        private final double kL; // geometry coefficient
        private final double kW;

        // Desired state for this loop
        private double targetSpeed = 0.0;  // 0–1
        private double targetAngle = 0.0;  // radians, field-relative

        // Steering PID state
        private double steerIntegral = 0.0;
        private double steerLastError = 0.0;

        SwerveModule(DcMotor drive, CRServo steer, AnalogInput encoder,
                     double encoderOffset, double kL, double kW) {
            this.drive = drive;
            this.steer = steer;
            this.encoder = encoder;
            this.encoderOffset = encoderOffset;
            this.kL = kL;
            this.kW = kW;
        }

        /**
         * Computes desired wheel speed and angle for given robot-frame vx, vy, omega.
         *
         * @return speed magnitude (for normalization across modules).
         */
        public double computeDesiredState(double vx, double vy, double omega) {
            double vxWheel = vx - omega * kL;
            double vyWheel = vy + omega * kW;

            double speed = Math.hypot(vxWheel, vyWheel);
            double angle = Math.atan2(vyWheel, vxWheel); // radians

            // Module optimization: compare with current angle and decide whether to flip 180°
            double currentAngle = getCurrentAngle();
            double delta = wrapAngle(angle - currentAngle);

            // If we’d need to turn more than 90°, flip direction
            if (Math.abs(delta) > Math.PI / 2.0) {
                angle = wrapAngle(angle + Math.PI);
                speed = -speed;
            }

            targetSpeed = speed;
            targetAngle = angle;
            return Math.abs(speed);
        }

        /** Apply steering PID and drive output. */
        public void apply(double dtSeconds, double maxSpeed) {
            // Normalize speed
            double drivePower = targetSpeed / maxSpeed;
            drivePower = clamp(drivePower, -1.0, 1.0);

            // Steering PID
            double currentAngle = getCurrentAngle();
            double error = wrapAngle(targetAngle - currentAngle);

            steerIntegral += error * dtSeconds;
            double derivative = (error - steerLastError) / dtSeconds;
            steerLastError = error;

            double steerPower =
                    STEER_KP * error + STEER_KI * steerIntegral + STEER_KD * derivative;
            steerPower = clamp(steerPower, -1.0, 1.0);

            // Apply to hardware
            drive.setPower(drivePower);
            steer.setPower(steerPower);
        }

        /** Current pod angle in radians, using analog encoder + offset. */
        private double getCurrentAngle() {
            double voltage = encoder.getVoltage();
            double angle = (voltage / ENCODER_MAX_VOLTAGE) * 2.0 * Math.PI;
            angle = wrapAngle(angle + encoderOffset);
            return angle;
        }
    }
}