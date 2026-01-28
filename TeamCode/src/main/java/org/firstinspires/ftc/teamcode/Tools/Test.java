package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
// IMU is no longer strictly needed for drive, but kept for initialization if mechanisms use it.
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Test", group = "Tools")
public class Test extends LinearOpMode {
    private Servo turretRotation1, turretRotation2;

    // Turret Rotation
    final double MIN_TURRET_ROTATION = 0.0;
    final double MAX_TURRET_ROTATION = 1.0;
    final double TURRET_ROTATION_STEP = 0.01;
    private double currentTurretRotation = (MIN_TURRET_ROTATION + MAX_TURRET_ROTATION)/2.0;

    @Override
    public void runOpMode() {

        initializeHardware();

        waitForStart();

        while (opModeIsActive()) {
            double turretRotate = -gamepad1.right_stick_x;
            currentTurretRotation += turretRotate * TURRET_ROTATION_STEP;
            currentTurretRotation = Math.max(MIN_TURRET_ROTATION, Math.max(currentTurretRotation, MAX_TURRET_ROTATION));//just clamping

            if (gamepad1.a) {
                currentTurretRotation = 0;
            }
            if (gamepad1.b) {
                currentTurretRotation = 1.0;
            }

            turretRotation1.setPosition(currentTurretRotation);
            turretRotation2.setPosition(1.0 - currentTurretRotation);

            turretRotation1.setPosition(currentTurretRotation);
            turretRotation2.setPosition(1.0 - currentTurretRotation);

            // --- TELEMETRY (Simplified) ---
            telemetry.addData("Turret Rotation:", "%.3f", currentTurretRotation);
            telemetry.update();
        }
    }

    // --- HELPER METHODS ---
    private void initializeHardware() {
        turretRotation1 = hardwareMap.get(Servo.class, "turret_rotation_1" );
        turretRotation2 = hardwareMap.get(Servo.class, "turret_rotation_2" );

        turretRotation1.setDirection(Servo.Direction.FORWARD);
        turretRotation2.setDirection(Servo.Direction.REVERSE);
    }
}