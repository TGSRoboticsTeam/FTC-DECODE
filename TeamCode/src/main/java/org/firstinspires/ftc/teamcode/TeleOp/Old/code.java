package org.firstinspires.ftc.teamcode.TeleOp.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.AnalogInput; // Removed: Cannot read angle encoders
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@TeleOp(name = "DiamondbackDriveFieldCentric_NO_ENCODERS", group = "Swerve")
public class code extends LinearOpMode {

    // --- 1. HARDWARE DECLARATIONS ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    // private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder; // Removed: Cannot read angle encoders
    private IMU imu;
    private DcMotor leftFly, rightFly, intake;
    private Servo trigger;
    private Servo adjuster;
    private Servo light;

    // --- 2. ROBOT GEOMETRY ---
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // --- 3. CRITICAL: OFFSETS (NO LONGER USED WITHOUT ENCODERS) ---
    /*
    final double FRONT_LEFT_OFFSET  = 5.2417;
    final double FRONT_RIGHT_OFFSET = 5.7881;
    final double BACK_LEFT_OFFSET   = 2.4143;
    final double BACK_RIGHT_OFFSET  = 4.8209;
    */

    // --- 4. TUNING PARAMETERS ---
    // final double STEER_KP = 0.6; // No longer used as steering PID is removed
    final double DRIVE_DEADBAND = 0.05;
    final double STEER_DEADBAND = 0.05;
    final double ADJUSTER_DEADBAND = 0.05;

    // --- 5. SPEED CONTROL CONSTANTS ---
    final double MAX_SPEED_GLOBAL = 0.8;
    final double MAX_SPEED_SLOW_MODE = 0.2;
    final double TRIGGER_THRESHOLD = 0.5;
    final double FULL_FLYWHEEL_POWER = 1.0;
    final double SLOW_FLYWHEEL_POWER = 0.75;

    // --- 6. MECHANISM DIRECTION REVERSAL (EASY ACCESS) ---
    final boolean LEFT_FLY_REVERSE    = false;
    final boolean RIGHT_FLY_REVERSE   = true;
    final boolean INTAKE_REVERSE      = true;

    // --- 7. SERVO PARAMETERS ---
    final double SWEEP_DOWN_POSITION = 0.33;
    final double SWEEP_UP_POSITION   = 0.05;
    final long SWEEP_DELAY_MS = 250;

    // Turret Tilt Control Parameters
    final double INITIAL_TURRET_TILT = 0.5;
    final double TURRET_TILT_STEP_STICK = 0.015;
    final double MIN_TURRET_TILT = 0.01;
    final double MAX_TURRET_TILT = 0.99;

    // --- 8. LIGHT SWEEP PARAMETERS ---
    final double LIGHT_DEFAULT_POSITION = 0.28;
    final double LIGHT_FULL_POWER_POSITION = 0.45;
    final double LIGHT_PARTIAL_POSITION = 0.388;

    // --- 9. TOGGLE STATE VARIABLES ---
    private boolean isCalibrationModeActive = false;
    private boolean rightStickButtonPreviouslyPressed = false;
    private boolean isFlywheelOn = false;
    private boolean isIntakeOn = false;
    private boolean leftTriggerPreviouslyPressed = false;
    private boolean rightTriggerPreviouslyPressed = false;
    private boolean aButtonPreviouslyPressed = false;
    private boolean yButtonPreviouslyPressed = false;
    private boolean isLowPowerMode = false;
    private boolean dpadLeftPreviouslyPressed = false;

    // Store last target angles (still needed for directionality, even if we can't enforce them)
    private double lastAngleFL = 0, lastAngleFR = 0, lastAngleBL = 0, lastAngleBR = 0;




    @Override
    public void runOpMode() {

        initializeHardware();

        // **CRITICAL**: Reset IMU Yaw on initialization so current direction is 0 degrees.
        imu.resetYaw();

        waitForStart();

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;
        double currentSpeedFL = 0, currentSpeedFR = 0, currentSpeedBL = 0, currentSpeedBR = 0;

        double turretTilt = INITIAL_TURRET_TILT;
        double currentFlywheelPower = 0.0;

        // Initialize servos
        trigger.setPosition(SWEEP_DOWN_POSITION);
        adjuster.setPosition(turretTilt);
        // Initialize light to the new default position
        light.setPosition(LIGHT_DEFAULT_POSITION);

        // --- EMERGENCY SWERVE STEER CONTROL (manual power adjustment) ---
        // Since PID steering is broken, use gamepad2 sticks to adjust steer CRServos manually
        // You would uncomment this block if you decide to try and manually set the angle.
        /*
        double manualSteerFL = 0, manualSteerFR = 0, manualSteerBL = 0, manualSteerBR = 0;
        */

        while (opModeIsActive()) {

            // --- INPUT READS ---
            boolean dpadLeft = gamepad1.dpad_left;

            // --- MANUAL SWERVE STEER ADJUSTMENT (Optional but necessary without encoders) ---
            /*
            manualSteerFL = gamepad2.left_stick_y;
            manualSteerFR = gamepad2.right_stick_y;
            // Map more buttons for the other two modules

            frontLeftSteer.setPower(manualSteerFL);
            frontRightSteer.setPower(manualSteerFR);
            // ... set other steer powers
            */


            // --- Toggle Logic for Calibration Mode (R3) ---
            boolean rightStickButtonCurrentlyPressed = gamepad1.right_stick_button;
            if (rightStickButtonCurrentlyPressed && !rightStickButtonPreviouslyPressed) {
                isCalibrationModeActive = !isCalibrationModeActive;
            }
            rightStickButtonPreviouslyPressed = rightStickButtonCurrentlyPressed;

            // Speed Limiter Logic
            double speedMultiplier = MAX_SPEED_GLOBAL;
            if (gamepad1.right_bumper) {
                speedMultiplier = MAX_SPEED_SLOW_MODE;
            }

            // --- YAW RESET LOGIC (Y Button) ---
            boolean yButtonCurrentlyPressed = gamepad1.y;
            if (yButtonCurrentlyPressed && !yButtonPreviouslyPressed) {
                imu.resetYaw();
                telemetry.addData("Field-Centric", "Yaw Reset to 0 degrees.");
                telemetry.update();
            }
            yButtonPreviouslyPressed = yButtonCurrentlyPressed;

            // Get current heading from IMU (in Radians)
            double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // --- CALIBRATION MODE CHECK (Disabled, as calibration now requires external tools) ---
            if (isCalibrationModeActive) {
                // Simplified calibration check (drive power is zeroed)
                frontLeftDrive.setPower(0); frontRightDrive.setPower(0);
                backLeftDrive.setPower(0); backRightDrive.setPower(0);

                // CRServos are set to 0 power (may drift)
                frontLeftSteer.setPower(0); frontRightSteer.setPower(0);
                backLeftSteer.setPower(0); backRightSteer.setPower(0);

                // Kill all mechanisms
                leftFly.setPower(0); rightFly.setPower(0); intake.setPower(0);

                telemetry.addData("Mode", "**MANUAL ALIGNMENT REQUIRED**");
                telemetry.addData("Warning", "Angle Encoders are offline. Steering CRServos are OFF.");
                telemetry.update();
                continue;
            }


            // --- TURRET TILT CONTROL (No change) ---
            double tiltInputStick = -gamepad2.right_stick_y;
            // ... (rest of tilt logic remains the same)

            // --- DRIVE INPUTS (FIELD-CENTRIC) ---

            double fieldY = -gamepad1.left_stick_y * speedMultiplier;
            double fieldX = gamepad1.left_stick_x * speedMultiplier;
            double rot = gamepad1.right_stick_x * speedMultiplier;

            double robotX = fieldX * Math.cos(-robotYaw) - fieldY * Math.sin(-robotYaw);
            double robotY = fieldX * Math.sin(-robotYaw) + fieldY * Math.cos(-robotYaw);

            double A = robotX - rot * (WHEELBASE / R);
            double B = robotX + rot * (WHEELBASE / R);
            double C = robotY - rot * (TRACK_WIDTH / R);
            double D = robotY + rot * (TRACK_WIDTH / R);

            currentSpeedFL = Math.hypot(B, D);
            currentSpeedFR = Math.hypot(B, C);
            currentSpeedBL = Math.hypot(A, D);
            currentSpeedBR = Math.hypot(A, C);

            // Normalize speeds
            double maxSpeed = Math.max(Math.max(currentSpeedFL, currentSpeedFR), Math.max(currentSpeedBL, currentSpeedBR));
            if (maxSpeed > 1.0) {
                currentSpeedFL /= maxSpeed;
                currentSpeedFR /= maxSpeed;
                currentSpeedBL /= maxSpeed;
                currentSpeedBR /= maxSpeed;
            }

            // Apply speeds directly to drive motors
            runDriveModule(frontLeftDrive, currentSpeedFL);
            runDriveModule(frontRightDrive, currentSpeedFR);
            runDriveModule(backLeftDrive, currentSpeedBL);
            runDriveModule(backRightDrive, currentSpeedBR);

            // --- CRITICAL: MANUAL STEER POWER ---
            // The CRServos are now set to 0, meaning the wheels are not actively being steered or held.
            // They rely on static friction. You must use the optional manual steer control above
            // if the angles drift during the match.
            frontLeftSteer.setPower(0.0);
            frontRightSteer.setPower(0.0);
            backLeftSteer.setPower(0.0);
            backRightSteer.setPower(0.0);


            // --- MECHANISM CONTROL (No change) ---
            // ... (All mechanism logic remains the same)

            // --- TELEMETRY ---
            telemetry.addData("Mode", "**FIXED-ANGLE DRIVE**");
            telemetry.addData("Yaw (Degrees)", "%.2f", Math.toDegrees(robotYaw));
            telemetry.addData("Steering", "Angle Encoders offline. Modules are fixed.");
            telemetry.update();
        }
    }

    // --- HELPER METHODS ---

    private void initializeHardware() {
        // ... (All hardware initialization, EXCEPT AnalogInput, remains the same)
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftSteer  = hardwareMap.get(CRServo.class, "frontLeftSteer");
        frontRightSteer = hardwareMap.get(CRServo.class, "frontRightSteer");
        backLeftSteer   = hardwareMap.get(CRServo.class, "backLeftSteer");
        backRightSteer  = hardwareMap.get(CRServo.class, "backRightSteer");
        // AnalogInput declarations removed
        imu = hardwareMap.get(IMU.class, "imu");

        // ... (Mechanism initialization, IMU initialization, Direction Fixes, Zero Power Behavior remains the same)
    }

    // Simplified runModule only handling drive power (Steering is disabled/manual)
    private void runDriveModule(DcMotor driveMotor, double speed) {
        driveMotor.setPower(speed);
        // Steering CRServo control is now handled manually or set to 0 power in the loop
    }

    // Deleted: runModule (PID steering logic)
    // Deleted: getRawAngle (Analog input reading)
    // Deleted: runCalibrationMode (Relied on AnalogInput)
    // ... (wrapAngle, resetMotors remain the same)
}