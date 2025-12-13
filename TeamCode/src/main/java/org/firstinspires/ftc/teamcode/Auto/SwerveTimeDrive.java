package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Auto.Shoot;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "SwerveTimeDrive", group = "Swerve")
public class SwerveTimeDrive extends LinearOpMode {

    // --- 1. HARDWARE DECLARATIONS (Copied from your TeleOp) ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;

    // shoot
    private Shoot shoot;
    // --- 2. ROBOT GEOMETRY ---
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // --- 3. OFFSETS (Your Measured Values) ---
    final double FRONT_LEFT_OFFSET  = 5.2417;
    final double FRONT_RIGHT_OFFSET = 5.7881;
    final double BACK_LEFT_OFFSET   = 2.4143;
    final double BACK_RIGHT_OFFSET  = 4.8209;

    // --- 4. TUNING PARAMETERS ---
    final double STEER_KP = 0.6;
    final double STEER_DEADBAND = 0.05;

    final long FLYWHEEL_SPINUP_MS = 2000;
    final long TIME_BETWEEN_SHOTS_MS = 3000;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData("Status", "Initialized. Waiting for Start...");
        telemetry.update();

        shoot = new Shoot(this, hardwareMap);

        waitForStart();

        // --------------------------------------------------------
        // COMMAND SEQUENCE
        // Format: driveRobotCentric(Strafe X, Forward Y, Turn, Seconds)
        // --------------------------------------------------------
        telemetry.addData("Auto", "1. Starting Shooting Sequence 3 shots");
        telemetry.update();

        shoot.shoot(
                3,
                FLYWHEEL_SPINUP_MS,
                TIME_BETWEEN_SHOTS_MS
        );

        // 1. Move BACKWARD at 50% speed for 1.5 seconds
        telemetry.addData("Auto", "Driving Forward");
        telemetry.update();
        driveRobotCentric(0, 0.6, 0, 2);
        stopDrive();



        telemetry.addData("Auto", "Complete");
        telemetry.update();
    }

    /**
     * core drive method for Autonomous.
     * Because your servos are CRServos acting as position servos via PID,
     * this method MUST run a loop to continuously update servo power.
     *
     * @param x       Strafe speed (-1.0 to 1.0)
     * @param y       Forward speed (-1.0 to 1.0)
     * @param rot     Turn speed (-1.0 to 1.0)
     * @param seconds Duration to run this movement
     */
    public void driveRobotCentric(double x, double y, double rot, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < seconds) {
            // Swerve Kinematics
            double A = x - rot * (WHEELBASE / R);
            double B = x + rot * (WHEELBASE / R);
            double C = y - rot * (TRACK_WIDTH / R);
            double D = y + rot * (TRACK_WIDTH / R);

            // Calculate speeds
            double speedFL = Math.hypot(B, D);
            double speedFR = Math.hypot(B, C);
            double speedBL = Math.hypot(A, D);
            double speedBR = Math.hypot(A, C);

            // Normalize speeds
            double maxSpeed = Math.max(Math.max(speedFL, speedFR), Math.max(speedBL, speedBR));
            if (maxSpeed > 1.0) {
                speedFL /= maxSpeed;
                speedFR /= maxSpeed;
                speedBL /= maxSpeed;
                speedBR /= maxSpeed;
            }

            // Calculate Target Angles
            double angleFL = Math.atan2(B, D);
            double angleFR = Math.atan2(B, C);
            double angleBL = Math.atan2(A, D);
            double angleBR = Math.atan2(A, C);

            // Apply to Modules
            runModule(frontLeftDrive, frontLeftSteer, frontLeftEncoder, FRONT_LEFT_OFFSET, speedFL, angleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFR, angleFR);
            runModule(backLeftDrive, backLeftSteer, backLeftEncoder, BACK_LEFT_OFFSET, speedBL, angleBL);
            runModule(backRightDrive, backRightSteer, backRightEncoder, BACK_RIGHT_OFFSET, speedBR, angleBR);
        }
    }

    private void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftSteer.setPower(0);
        frontRightSteer.setPower(0);
        backLeftSteer.setPower(0);
        backRightSteer.setPower(0);
    }

    // --- HELPER METHODS (Reused directly from your TeleOp) ---

    private void runModule(DcMotor driveMotor, CRServo steerServo, AnalogInput encoder, double encoderOffset, double speed, double targetAngle) {
        double rawAngle = getRawAngle(encoder);
        double currentAngle = rawAngle - encoderOffset;
        currentAngle = wrapAngle(currentAngle);

        double delta = wrapAngle(targetAngle - currentAngle);

        // Optimization: Flip wheel direction if turn is > 90 degrees
        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        // PID Control for Continuous Servo
        double servoPower = STEER_KP * delta;
        servoPower *= -1; // Invert to match physical rotation

        if (Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0;
        servoPower = Math.max(-1, Math.min(1, servoPower));

        steerServo.setPower(servoPower);
        driveMotor.setPower(speed);
    }

    private double getRawAngle(AnalogInput encoder) {
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

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

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}

/*
Gemini's moving code:
// 1. Move FORWARD at 50% speed for 1.5 seconds
        telemetry.addData("Auto", "Driving Forward");
        telemetry.update();
        driveRobotCentric(0, 0.5, 0, 1.5);
        stopDrive();
        sleep(500); // Short pause between moves

        // 2. Move BACKWARD at 50% speed for 1.5 seconds
        telemetry.addData("Auto", "Driving Backward");
        telemetry.update();
        driveRobotCentric(0, -0.5, 0, 1.5);
        stopDrive();
        sleep(500);

        // 3. Move RIGHT at 50% speed for 1.5 seconds
        telemetry.addData("Auto", "Driving Right");
        telemetry.update();
        driveRobotCentric(0.5, 0, 0, 1.5);
        stopDrive();
        sleep(500);

        // 4. Move LEFT at 50% speed for 1.5 seconds
        telemetry.addData("Auto", "Driving Left");
        telemetry.update();
        driveRobotCentric(-0.5, 0, 0, 1.5);
        stopDrive();
 */