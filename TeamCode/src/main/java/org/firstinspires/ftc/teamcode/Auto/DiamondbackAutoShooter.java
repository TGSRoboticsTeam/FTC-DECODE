package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

// IMPORTANT: This op mode is based on the logic from your TeleOp but removes the drive functionality
// since only the mechanism control is required.

@Autonomous(name = "Diamondback Auto Shooter", group = "Autonomous")
public class DiamondbackAutoShooter extends LinearOpMode {

    // --- 1. HARDWARE DECLARATIONS (Mechanism only) ---
    private DcMotor leftFly, rightFly, intake;
    private Servo trigger;
    private Servo adjuster;
    private Servo light;
    private IMU imu;

    // We keep the drive/steer hardware declarations simple (or remove them) since they won't be used.
    // For simplicity, we'll keep the mechanism-only declarations.

    // --- 2. MECHANISM CONSTANTS (Copied from TeleOp) ---
    final boolean LEFT_FLY_REVERSE    = false;
    final boolean RIGHT_FLY_REVERSE   = true;
    final boolean INTAKE_REVERSE      = true;

    final double SWEEP_DOWN_POSITION = 0.33;
    final double SWEEP_UP_POSITION   = 0.05;
    final long SWEEP_DELAY_MS = 250; // Time the servo stays up to push the ring

    // --- 3. AUTONOMOUS TIMING CONSTANTS ---
    final long FLYWHEEL_SPINUP_MS = 2000; // 2.0 seconds for flywheel to reach full speed
    final long TIME_BETWEEN_SHOTS_MS = 3000; // 3.0 seconds between each shot
    final int NUM_SHOTS = 3;

    @Override
    public void runOpMode() {

        initializeHardware();

        // Initialize servos to their rest state
        trigger.setPosition(SWEEP_DOWN_POSITION);
        // Adjuster and Light are initialized but do not need to be actively controlled
        // unless you want a specific autonomous angle.

        telemetry.addData("Status", "Hardware Initialized. Waiting for Start.");
        telemetry.update();

        waitForStart();

        // --- AUTONOMOUS SEQUENCE START ---

        if (opModeIsActive()) {

            // 1. TURN ON FLYWHEEL AND INTAKE
            telemetry.addData("Status", "1. Starting Flywheels and Intake.");
            telemetry.update();
            leftFly.setPower(1.0);
            rightFly.setPower(1.0);
            intake.setPower(1.0);

            // 2. WAIT FOR FLYWHEEL SPIN-UP
            sleep(FLYWHEEL_SPINUP_MS);

            // 3. SHOOT RINGS (3 times)
            for (int i = 0; i < NUM_SHOTS && opModeIsActive(); i++) {

                // Fire one shot
                fireRing();
                telemetry.addData("Status", "3. Fired Shot %d of %d", i + 1, NUM_SHOTS);
                telemetry.update();

                // Wait the required delay between shots
                if (i < NUM_SHOTS - 1) { // Don't wait after the last shot
                    sleep(TIME_BETWEEN_SHOTS_MS);
                }
            }

            // 4. STOP MECHANISMS
            telemetry.addData("Status", "4. Stopping Mechanisms. Auto Complete.");
            telemetry.update();
            leftFly.setPower(0);
            rightFly.setPower(0);
            intake.setPower(0);
        }
    }

    // --- HELPER METHOD TO EXECUTE ONE SHOT ---
    private void fireRing() {
        trigger.setPosition(SWEEP_UP_POSITION);
        sleep(SWEEP_DELAY_MS);
        trigger.setPosition(SWEEP_DOWN_POSITION);
    }

    // --- HARDWARE INITIALIZATION ---
    private void initializeHardware() {
        // --- Mechanism Hardware (Launcher/Intake only) ---
        leftFly = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");
        intake = hardwareMap.get(DcMotor.class, "intake");
        trigger = hardwareMap.get(Servo.class, "trigger");
        adjuster = hardwareMap.get(Servo.class, "adjuster");
        light = hardwareMap.get(Servo.class, "light");
        imu = hardwareMap.get(IMU.class, "imu"); // Required for full TeleOp compatibility, even if not used here.

        // --- Motor Direction Fix (From TeleOp) ---
        leftFly.setDirection(LEFT_FLY_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rightFly.setDirection(RIGHT_FLY_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setDirection(INTAKE_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // Set Zero Power Behavior
        leftFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure motors are running without encoders for continuous power control
        leftFly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFly.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU (Even if not used for movement, it's good practice for consistency)
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(logoDirection, usbDirection)
        );
        imu.initialize(parameters);

        // Note: Drive motors and steer servos were excluded here to keep the code clean
        // for a pure shooting autonomous.
    }
}