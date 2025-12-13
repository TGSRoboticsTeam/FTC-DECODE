package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Encapsulates the complete shooter autonomous routine.
 * Requires the active LinearOpMode to access sleep() and opModeIsActive().
 */
public class DiamondbackAutoMechanism {

    // --- Hardware Declarations ---
    private DcMotor leftFly, rightFly, intake;
    private Servo trigger;

    // --- Constant Declarations (Used in the method) ---
    final double SWEEP_DOWN_POSITION = 0.33;
    final double SWEEP_UP_POSITION = 0.05;
    final long SWEEP_DELAY_MS = 250;

    // --- Reference to the OpMode ---
    private final LinearOpMode opMode;

    // --- Constructor: Initializes Hardware and Stores OpMode Reference ---
    public DiamondbackAutoMechanism(LinearOpMode activeOpMode, HardwareMap hwMap) {
        // Store the reference to the running OpMode
        this.opMode = activeOpMode;

        // --- Hardware Mapping (Simplified, assuming initialization is done) ---
        leftFly = hwMap.get(DcMotor.class, "leftFly");
        rightFly = hwMap.get(DcMotor.class, "rightFly");
        intake = hwMap.get(DcMotor.class, "intake");
        trigger = hwMap.get(Servo.class, "trigger");

        // Ensure initial position is set
        trigger.setPosition(SWEEP_DOWN_POSITION);

        // NOTE: Full motor configuration (direction, mode) is assumed to be handled here
        // or in a separate initialization method called from the OpMode.
    }

    // --- Helper Method (Original fireRing logic) ---
    private void fireRing() {
        trigger.setPosition(SWEEP_UP_POSITION);
        opMode.sleep(SWEEP_DELAY_MS); // Use opMode.sleep()
        trigger.setPosition(SWEEP_DOWN_POSITION);
    }


    // --- THE MAIN METHOD YOU WANTED ---
    /**
     * Executes the full sequential shooter routine.
     * @param NUM_SHOTS The number of rings to fire.
     * @param FLYWHEEL_SPINUP_MS Time to wait for spin up.
     * @param TIME_BETWEEN_SHOTS_MS Delay between shots.
     */
    public void shoot(
            final int NUM_SHOTS,
            final long FLYWHEEL_SPINUP_MS,
            final long TIME_BETWEEN_SHOTS_MS) {

        // Check if the OpMode is still running before starting
        if (!opMode.opModeIsActive()) {
            return;
        }

        // 1. TURN ON FLYWHEEL AND INTAKE
        opMode.telemetry.addData("Status", "1. Starting Flywheels and Intake.");
        opMode.telemetry.update();
        leftFly.setPower(1.0);
        rightFly.setPower(1.0);
        intake.setPower(1.0);

        // 2. WAIT FOR FLYWHEEL SPIN-UP
        opMode.sleep(FLYWHEEL_SPINUP_MS); // Use opMode.sleep()

        // 3. SHOOT RINGS (3 times)
        for (int i = 0; i < NUM_SHOTS && opMode.opModeIsActive(); i++) { // Use opMode.opModeIsActive()

            // Fire one shot
            fireRing();
            opMode.telemetry.addData("Status", "3. Fired Shot %d of %d", i + 1, NUM_SHOTS);
            opMode.telemetry.update();

            // Wait the required delay between shots
            if (i < NUM_SHOTS - 1) {
                opMode.sleep(TIME_BETWEEN_SHOTS_MS); // Use opMode.sleep()
            }
        }

        // Stop mechanisms after routine is complete
        leftFly.setPower(0);
        rightFly.setPower(0);
        intake.setPower(0);
    }
}