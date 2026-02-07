package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.math.Vector;

@TeleOp(name = "Swerve: Real-Time Encoder Diagnostic", group = "Swerve")
public class EncoderDiagnostic extends LinearOpMode {

    private SwerveDrivetrain drivetrain;

    @Override
    public void runOpMode() {
        drivetrain = new SwerveDrivetrain(hardwareMap);

        telemetry.addData("Status", "Initialized. Resetting pods...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // CRITICAL: We must call a method that triggers an encoder read.
            // calculateDrive processes the vectors and updates the internal targetAngles array.
            drivetrain.calculateDrive(new Vector(0,0), new Vector(0,0), new Vector(0,0), 0);

            // Now we display the updated values
            telemetry.addLine("--- Pod Angles (Radians) ---");
            // debugString pulls the current voltage from the encoders
            telemetry.addLine(drivetrain.debugString());

            telemetry.addLine("\n--- Diagnostics ---");
            telemetry.addData("Voltage Sensor", drivetrain.getVoltage());
            telemetry.addLine("Rotate pods manually to see Raw values change.");

            telemetry.update();
        }
    }
}