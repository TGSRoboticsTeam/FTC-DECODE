package org.firstinspires.ftc.teamcode.TestingTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * Test OpMode for diagnosing the Front-Left Steering Servo (CRServo) and its encoder.
 * * Instructions:
 * - Use the LEFT STICK Y to directly set the power of the frontLeftSteer CRServo.
 * - Observe the encoder voltage/angle to verify the sensor is working.
 */
@Disabled
@TeleOp(name = "FL_Steering_Test", group = "Tests")
public class FL_Steering_Test extends LinearOpMode {

    // --- 1. HARDWARE DECLARATIONS ---
    private CRServo frontLeftSteer;
    private AnalogInput frontLeftEncoder;

    // --- 2. CONSTANTS ---
    final double FRONT_LEFT_OFFSET = 5.2417; // Re-use the offset from your main code

    @Override
    public void runOpMode() {

        // --- 3. HARDWARE INITIALIZATION ---
        try {
            frontLeftSteer = hardwareMap.get(CRServo.class, "frontLeftSteer");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find frontLeftSteer. Check config.");
            telemetry.update();
            sleep(5000);
            return;
        }

        try {
            frontLeftEncoder = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find frontLeftEncoder. Check config.");
            telemetry.update();
            sleep(5000);
            return;
        }

        telemetry.addData("Status", "Hardware Initialized. Waiting for Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 4. INPUT CONTROL ---
            // Left stick Y controls the steering servo power directly.
            // Move stick up for max forward power (1.0), down for max reverse power (-1.0).
            double servoPower = -gamepad1.left_stick_y;

            // Apply deadband to prevent drift/jitter when stick is centered
            final double STICK_DEADBAND = 0.05;
            if (Math.abs(servoPower) < STICK_DEADBAND) {
                servoPower = 0.0;
            }

            frontLeftSteer.setPower(servoPower);

            // --- 5. ENCODER READING ---
            double rawVoltage = frontLeftEncoder.getVoltage();
            double rawAngle = getRawAngle(frontLeftEncoder);
            double currentAngle = rawAngle - FRONT_LEFT_OFFSET;
            currentAngle = wrapAngle(currentAngle);


            // --- 6. TELEMETRY OUTPUT ---
            telemetry.addData("Mode", "FL Steering Test");
            telemetry.addData("Instructions", "Use LEFT STICK Y to control servo power.");
            telemetry.addData("---------------------", "---------------------");
            telemetry.addData("1. Servo Power", "%.2f", servoPower);
            telemetry.addData("2. Servo Name", "frontLeftSteer");
            telemetry.addData("3. Encoder Voltage", "%.4f V", rawVoltage);
            telemetry.addData("4. Raw Angle (Rad)", "%.4f", rawAngle);
            telemetry.addData("5. Angle (Corrected)", "%.4f", currentAngle);
            telemetry.addData("6. Used Offset", "%.4f", FRONT_LEFT_OFFSET);
            telemetry.update();
        }
    }

    // --- HELPER METHODS (Copied from DiamondbackDrive.java) ---

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double getRawAngle(AnalogInput encoder) {
        // Assuming 3.3V is the max analog voltage
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }
}