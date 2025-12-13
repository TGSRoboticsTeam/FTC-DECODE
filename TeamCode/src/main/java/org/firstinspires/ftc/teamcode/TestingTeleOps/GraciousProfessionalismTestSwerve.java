package org.firstinspires.ftc.teamcode.TestingTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@TeleOp(name = "GPTSwerve", group = "Swerve")
public class GraciousProfessionalismTestSwerve extends LinearOpMode { // G.P.T.Swerve, you're welcome.

    // Drive Motors
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // Steering Servos
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;

    // Analog Encoders (absolute angle sensors)
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;

    // IMU for field-centric orientation
    private IMU imu;

    // Robot Geometry (in inches)
    final double TRACK_WIDTH = 17.258;    // Width between left and right wheels
    final double WHEELBASE   = 13.544;    // Length between front and back wheels
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE); // Diagonal distance for swerve math

    // Offsets for CRServo encoders
    final double FRONT_LEFT_OFFSET = 2.796;
    final double FRONT_RIGHT_OFFSET = 3.059;
    final double BACK_LEFT_OFFSET = 1.280;
    final double BACK_RIGHT_OFFSET = 2.538;


    // Optional Auto-align target (X axis, in inches)
    double targetX = 0;

    @Override
    public void runOpMode() {

        // Hardware Mapping
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

        // Initialize IMU with REV Hub orientation parameters
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        // Reset drive motors
        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);

        waitForStart();

        // Heading offset for zeroing field-centric orientation
        double headingOffset = 0;

        while (opModeIsActive()) {

            // Zero Heading (press START button)
            if (gamepad1.start) {
                headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            // Current heading in radians, corrected by zero offset
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;

            // Driving Input
            double y = -gamepad1.left_stick_y; // Forward/backward
            double x = gamepad1.left_stick_x;  // Left/right
            double rot = gamepad1.right_stick_x; // Rotation

            // Field-centric transformation
            double cosA = Math.cos(heading);
            double sinA = Math.sin(heading);
            double robotX = x * cosA - y * sinA;
            double robotY = x * sinA + y * cosA;

            // Optional Auto-align X axis
            if (gamepad1.left_bumper) {
                double error = targetX - robotX; // Proportional correction
                robotX = error * 0.5;
            }

            // Lock wheels in place (nullify opponent defense)
            boolean lockedInPlace = gamepad1.left_stick_button;

            // Swerve Kinematics
            double A = robotX - rot * (WHEELBASE / R);
            double B = robotX + rot * (WHEELBASE / R);
            double C = robotY - rot * (TRACK_WIDTH / R);
            double D = robotY + rot * (TRACK_WIDTH / R);

            // Calculate wheel speeds
            double speedFrontLeft  = Math.hypot(B, D);
            double speedFrontRight = Math.hypot(B, C);
            double speedBackLeft   = Math.hypot(A, D);
            double speedBackRight  = Math.hypot(A, C);

            double maxSpeed = Math.max(Math.max(speedFrontLeft, speedFrontRight), Math.max(speedBackLeft, speedBackRight));
            if (maxSpeed > 1.0) {
                speedFrontLeft /= maxSpeed;
                speedFrontRight /= maxSpeed;
                speedBackLeft /= maxSpeed;
                speedBackRight /= maxSpeed;
            }

            // Calculate wheel angles
            double angleFrontLeft  = Math.atan2(B, D);
            double angleFrontRight = Math.atan2(B, C);
            double angleBackLeft   = Math.atan2(A, D);
            double angleBackRight  = Math.atan2(A, C);

            if (lockedInPlace) {
                angleFrontLeft  += Math.PI / 4;  // 45°
                angleFrontRight -= Math.PI / 4;  // -45°
                angleBackLeft   -= Math.PI / 4;  // -45°
                angleBackRight  += Math.PI / 4;  // 45°

                speedFrontLeft  = 0;
                speedFrontRight = 0;
                speedBackLeft   = 0;
                speedBackRight  = 0;
            }

            // Apply module outputs
            runModule(frontLeftDrive, frontLeftSteer, frontLeftEncoder, FRONT_LEFT_OFFSET, speedFrontLeft, angleFrontLeft);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, angleFrontRight);
            runModule(backLeftDrive, backLeftSteer, backLeftEncoder, BACK_LEFT_OFFSET, speedBackLeft, angleBackLeft);
            runModule(backRightDrive, backRightSteer, backRightEncoder, BACK_RIGHT_OFFSET, speedBackRight, angleBackRight);

            // Telemetry for debugging
            telemetry.addData("Heading (deg)", Math.toDegrees(heading));
            telemetry.addData("Target X", targetX);
            telemetry.addData("Joystick X", x);
            telemetry.update();
        }
    }

    // Module Control (Steering + Wheel)
    // Module Control (Steering + Wheel) with encoder offsets
    private void runModule(DcMotor driveMotor, CRServo steerServo, AnalogInput encoder, double encoderOffset, double speed, double targetAngle) {

        // Read absolute encoder (0-2PI) and apply offset
        double currentAngle = encoder.getVoltage() / 3.3 * (2 * Math.PI) - encoderOffset;
        currentAngle = wrapAngle(currentAngle);

        // Compute error between target and current
        double delta = wrapAngle(targetAngle - currentAngle);

        // 180° optimization to minimize steering rotation
        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1; // Reverse wheel direction
        }

        // Proportional controller for CRServo
        double kP = 2.0; // Adjust experimentally
        double servoPower = kP * delta;

        // Deadband to prevent oscillation
        if (Math.abs(servoPower) < 0.05) servoPower = 0;

        // Clip servo power to [-1, 1]
        servoPower = Math.max(-1, Math.min(1, servoPower));

        // Apply powers
        steerServo.setPower(servoPower);
        driveMotor.setPower(speed);
    }


    // Helper: Normalize angle to [-PI, PI]
    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // Helper: Reset drive motors
    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // Appendix
    /*
    1. Swerve Math:
       - A, B, C, D: Wheel vector components for rotation + translation
       - speed: hypotenuse of vector (magnitude)
       - angle: atan2(B,D) etc. (direction)

    2. Field-Centric Control:
       - Robot-oriented joystick input rotated by IMU heading
       - Cosine/sine used to rotate vector into world coordinates

    3. 180° Optimization:
       - If steering delta > 90°, rotate wheel 180° and invert drive
       - Minimizes servo rotation time, improves responsiveness

    4. IMU:
       - Zero heading via pressing START
       - Used to rotate joystick input for field-centric control

    5. Naming Conventions:
       - frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive
       - frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer
       - frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder

    6. Telemetry:
       - Heading in degrees
       - Target X (for auto-align)
       - Joystick X for driver reference

    7. Notes:
       -
    */
}
