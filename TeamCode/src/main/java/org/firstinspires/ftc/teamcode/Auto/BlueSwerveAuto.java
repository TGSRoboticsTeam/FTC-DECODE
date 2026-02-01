package org.firstinspires.ftc.teamcode.Auto;


import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

// Imports for Pedro Pathing are commented out as the functions use direct Swerve control
// import com.pedropathing.follower.Follower;
// import org.firstinspires.ftc.teamcode.Auto.pedroPathing.Constants;

@Autonomous(name = "BlueSwerveAuto", group = "Swerve_Auto")
@Disabled
public class BlueSwerveAuto extends LinearOpMode {

    // --- 1. HARDWARE DECLARATIONS (Copied from TeleOp) ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;
    private DcMotor leftFly, rightFly, intake;
    private Servo trigger;
    private Servo adjuster;
    private Servo light;

    // --- 2. ROBOT GEOMETRY (Copied from TeleOp) ---
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // --- 3. CRITICAL: OFFSETS (Copied from TeleOp) ---
    final double FRONT_LEFT_OFFSET  = 5.2417; // This value is in radians (0 to 2π)
    final double FRONT_RIGHT_OFFSET = 5.7881;
    final double BACK_LEFT_OFFSET   = 2.4143;
    final double BACK_RIGHT_OFFSET  = 4.8209;

    // --- 4. TUNING PARAMETERS (Copied from TeleOp) ---
    final double STEER_KP = 0.6;
    final double STEER_DEADBAND = 0.05;

    // --- 5. MECHANISM CONTROL CONSTANTS ---
    final double FULL_FLYWHEEL_POWER = 1.0;
    final double SWEEP_DOWN_POSITION = 0.33;
    final double SWEEP_UP_POSITION   = 0.05;
    final long SWEEP_DELAY_MS = 250;

    // --- 6. MECHANISM DIRECTION REVERSAL (Copied from TeleOp) ---
    final boolean LEFT_FLY_REVERSE    = false;
    final boolean RIGHT_FLY_REVERSE   = true;
    final boolean INTAKE_REVERSE      = true;

    // --- 7. AUTO MOVEMENT CONSTANTS ---
    // Max movement power (adjust this for speed in autonomous)
    final double AUTO_MOVE_POWER = 0.5;
    // This velocity is an estimate for calculating time from distance (inches/second).
    // Must be tuned experimentally for accuracy!
    final double ESTIMATED_VELOCITY_IN_PER_SEC = 25.0;

    // Timer is used for time-based movement
    private com.pedropathing.util.Timer autoTimer;


    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        imu.resetYaw();
        autoTimer = new com.pedropathing.util.Timer();

        // Ensure initial positions for mechanisms
        trigger.setPosition(SWEEP_DOWN_POSITION);
        // adjuster.setPosition(...) - assumes a predetermined auto angle is set elsewhere

        telemetry.addData("Status", "Hardware Initialized. Ready to Run.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // --- AUTONOMOUS SEQUENCE ---

        // Example: Run the layup routine
        telemetry.addData("Status", "Running Layup Routine...");
        telemetry.update();
        Timer timer2 = new Timer();
        timer2.resetTimer();
        while(timer2.getElapsedTimeSeconds() < 15){
        }
        blueLayup();

        // Example: Run the threePointer routine
        // telemetry.addData("Status", "Running Three Pointer Routine...");
        // telemetry.update();
        // threePointer();

        telemetry.addData("Status", "Autonomous Routine Complete.");
        telemetry.update();
    }


    /**
     * Autonomous function one: Layup Shot
     * Moves back for 2 seconds (or ~3 feet), shoots 3 balls, then moves right for 2 feet.
     */

    public void blueLayup() {
        // --- 1. Move Back for 2 Seconds (Target: 3 feet / 36 inches) ---
        // 36 inches / 25 in/s = 1.44 seconds. Using the requested 2 seconds for slower/more powerful movement.
        moveSwerveTime(0, -AUTO_MOVE_POWER, 0, 1000); // fieldY is negative for moving back

        // Stop movement
        moveSwerve(0, 0, 0, 0);

        // --- 2. Shoot 3 Balls ---
        shootBalls(3);

        // --- 3. Move Right for 2 Feet (Target: 24 inches) ---
        // 24 inches / 25 in/s = 0.96 seconds. Use 1000ms (1 second) for simplicity.
        moveSwerveTime(-AUTO_MOVE_POWER, 0, 0, 1250); // fieldX is positive for moving right

        // Stop movement
        moveSwerve(0, 0, 0, 0);

    }

    /**
     * Autonomous function two: Three Pointer Shot
     * Shoots 3 balls, then moves up by 2 feet.
     */
    public void threePointer() {
        // --- 1. Shoot 3 Balls ---
        shootBalls(3);

        // --- 2. Move Up by 2 Feet (Target: 24 inches) ---
        // 24 inches / 25 in/s = 0.96 seconds. Use 1000ms (1 second) for simplicity.
        moveSwerveTime(0, AUTO_MOVE_POWER, 0, 1000); // fieldY is positive for moving up

        // Stop movement
        moveSwerve(0, 0, 0, 0);
    }

    // =========================================================================
    // --- SWERVE MOVEMENT HELPER FUNCTIONS ---
    // =========================================================================

    /**
     * Commands the swerve drive to move for a specified duration.
     * @param fieldX Field-centric forward/back power (-1.0 to 1.0)
     * @param fieldY Field-centric left/right power (-1.0 to 1.0)
     * @param rot Rotation power (-1.0 to 1.0)
     * @param durationMs Time in milliseconds to run the movement
     */
    private void moveSwerveTime(double fieldX, double fieldY, double rot, long durationMs) {
        autoTimer.resetTimer();
        while (autoTimer.getElapsedTime() < durationMs && opModeIsActive()) {
            // Continuously update the swerve modules
            moveSwerve(fieldX, fieldY, rot, 0.0);
            telemetry.addData("Status", "Moving for time...");
            telemetry.addData("Time Left (s)", (durationMs - autoTimer.getElapsedTime()) / 1000.0);
            telemetry.update();
        }
        // Stop movement after time is up
        moveSwerve(0, 0, 0, 0);
    }

    /**
     * Core swerve movement logic (adapted from TeleOp) to calculate motor powers and steering angles.
     * @param fieldX Field-centric X power
     * @param fieldY Field-centric Y power
     * @param rot Rotation power
     * @param currentYaw Current robot heading in Radians (0.0 for field-centric auto)
     */
    private void moveSwerve(double fieldX, double fieldY, double rot, double currentYaw) {
        // --- DRIVE INPUTS (FIELD-CENTRIC) ---
        double robotX = fieldX * Math.cos(-currentYaw) - fieldY * Math.sin(-currentYaw);
        double robotY = fieldX * Math.sin(-currentYaw) + fieldY * Math.cos(-currentYaw);

        double A = robotX - rot * (WHEELBASE / R);
        double B = robotX + rot * (WHEELBASE / R);
        double C = robotY - rot * (TRACK_WIDTH / R);
        double D = robotY + rot * (TRACK_WIDTH / R);

        double speedFrontLeft  = Math.hypot(B, D);
        double speedFrontRight = Math.hypot(B, C);
        double speedBackLeft   = Math.hypot(A, D);
        double speedBackRight  = Math.hypot(A, C);

        double targetAngleFL = Math.atan2(B, D);
        double targetAngleFR = Math.atan2(B, C);
        double targetAngleBL = Math.atan2(A, D);
        double targetAngleBR = Math.atan2(A, C);

        // Normalize speeds
        double maxSpeed = Math.max(Math.max(speedFrontLeft, speedFrontRight), Math.max(speedBackLeft, speedBackRight));
        if (maxSpeed > 1.0) {
            speedFrontLeft /= maxSpeed;
            speedFrontRight /= maxSpeed;
            speedBackLeft /= maxSpeed;
            speedBackRight /= maxSpeed;
        }

        runModule(frontLeftDrive, frontLeftSteer, frontLeftEncoder, FRONT_LEFT_OFFSET, speedFrontLeft, targetAngleFL);
        runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR);
        runModule(backLeftDrive, backLeftSteer, backLeftEncoder, BACK_LEFT_OFFSET, speedBackLeft, targetAngleBL);
        runModule(backRightDrive, backRightSteer, backRightEncoder, BACK_RIGHT_OFFSET, speedBackRight, targetAngleBR);
    }

    /**
     * Executes the shooting sequence for a specified number of balls.
     * @param count The number of balls to shoot.
     */
    public void shootBalls(int count) {

        intake.setPower(1);
        adjuster.setPosition(0.441);
        // 1. Turn on the flywheel to full power
        leftFly.setPower(0.75*FULL_FLYWHEEL_POWER);
        rightFly.setPower(0.75*FULL_FLYWHEEL_POWER);



        // Wait for the flywheel to spin up (DWELL time)
        sleep(1000); // 1.0 second dwell time (tune this value)


        // 2. Fire the balls
        for (int i = 0; i < count; i++) {
            if (!opModeIsActive()) break;




            telemetry.addData("Shooting", "Ball %d of %d", i + 1, count);
            telemetry.update();

            // Actuate the trigger servo to sweep up
            trigger.setPosition(SWEEP_UP_POSITION);
            sleep(SWEEP_DELAY_MS);

            // Return the trigger servo to the down position
            trigger.setPosition(SWEEP_DOWN_POSITION);
            sleep(SWEEP_DELAY_MS);


        }

        // 3. Turn off the flywheel
        leftFly.setPower(0);
        rightFly.setPower(0);
        intake.setPower(0);

        telemetry.addData("Shooting", "Complete.");
        telemetry.update();
    }

    // =========================================================================
    // --- HARDWARE & UTILITY FUNCTIONS (Copied from TeleOp) ---
    // =========================================================================

    private void initializeHardware() {
        // --- Swerve Drive Hardware ---
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

        // --- Mechanism Hardware ---
        leftFly = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");
        intake = hardwareMap.get(DcMotor.class, "intake");
        trigger = hardwareMap.get(Servo.class, "trigger");
        adjuster = hardwareMap.get(Servo.class, "adjuster");
        light = hardwareMap.get(Servo.class, "light");

        // **CRITICAL**: Define the orientation of the Hub on the robot.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(logoDirection, usbDirection)
        );
        imu.initialize(parameters);

        // --- Swerve Drive Motor Direction Fix ---
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // --- Mechanism Motor Direction Fix ---
        leftFly.setDirection(LEFT_FLY_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rightFly.setDirection(RIGHT_FLY_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setDirection(INTAKE_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // Set Zero Power Behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset RunMode for all DC motors
        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, leftFly, rightFly, intake);
    }

    private void runModule(DcMotor driveMotor, CRServo steerServo, AnalogInput encoder, double encoderOffset, double speed, double targetAngle) {
        // Swerve module control logic (PID loop for steering)
        double rawAngle = getRawAngle(encoder);
        double currentAngle = rawAngle - encoderOffset;
        currentAngle = wrapAngle(currentAngle);

        double delta = wrapAngle(targetAngle - currentAngle);

        // Check for 180-degree flip optimization
        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        double servoPower = STEER_KP * delta;
        servoPower *= -1; // Steering Fix: Invert servo power to match physical rotation

        if (Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0;

        servoPower = Math.max(-1, Math.min(1, servoPower));

        steerServo.setPower(servoPower);
        driveMotor.setPower(speed);
    }

    private double wrapAngle(double angle) {
        // Wraps angle between -PI and PI
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private double getRawAngle(AnalogInput encoder) {
        // Converts Analog Input voltage (0-3.3V) to Radians (0 to 2π)
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }
}