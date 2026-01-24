package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
@Disabled
@TeleOp(name = "CalibrateSwerveEncoders", group = "Swerve")
public class CalibrateSwerveEncoders extends LinearOpMode {
    // Steering Servos
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;

    // Analog Encoders (absolute angle sensors)
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;

    @Override
    public void runOpMode() {
        frontLeftSteer  = hardwareMap.get(CRServo.class, "frontLeftSteer");
        frontRightSteer = hardwareMap.get(CRServo.class, "frontRightSteer");
        backLeftSteer   = hardwareMap.get(CRServo.class, "backLeftSteer");
        backRightSteer  = hardwareMap.get(CRServo.class, "backRightSteer");

        frontLeftEncoder  = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder   = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder  = hardwareMap.get(AnalogInput.class, "backRightEncoder");

        CRServo selectedModule = null;

        waitForStart();

        while (opModeIsActive()) {

            // Selecting the module
            if (gamepad1.a) {
                selectedModule = frontLeftSteer;
            }
            if (gamepad1.b) {
                selectedModule = frontRightSteer;
            }
            if (gamepad1.x) {
                selectedModule = backLeftSteer;
            }
            if (gamepad1.y) {
                selectedModule = backRightSteer;
            }

            // Setting the power
            boolean dpadDown = gamepad1.dpad_down;
            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;
            boolean dpadUp = gamepad1.dpad_up;

            double smallest = 0.005;
            double sMedium = 0.01;
            double lMedium = 0.1;
            double largest = 0.3;

            double power = 0.0;
            if (dpadDown) {
                power = smallest;
            } else if (dpadLeft) {
                power = sMedium;
            } else if (dpadRight) {
                power = lMedium;
            } else if (dpadUp) {
                power = largest;
            }

            frontLeftSteer.setPower(0);
            frontRightSteer.setPower(0);
            backLeftSteer.setPower(0);
            backRightSteer.setPower(0);
            if (selectedModule != null) {
                selectedModule.setPower(power);
            }

            String selectedText = "None";
            if (selectedModule == frontLeftSteer) {
                selectedText = "frontLeftModule";
            } else if (selectedModule == frontRightSteer) {
                selectedText = "frontRightModule";
            } else if (selectedModule == backLeftSteer) {
                selectedText = "backLeftModule";
            } else if (selectedModule == backRightSteer) {
                selectedText = "backRightModule";
            } else {
                telemetry.addData("Please select a module (press A, B, X, Y)", "");
                telemetry.addData("", "");
            }

            telemetry.addData("Selected Module:", selectedText);
            telemetry.addData("frontLeftEncoder", frontLeftEncoder.getVoltage());
            telemetry.addData("frontRightEncoder", frontRightEncoder.getVoltage());
            telemetry.addData("backLeftEncoder", backLeftEncoder.getVoltage());
            telemetry.addData("backRightEncoder", backRightEncoder.getVoltage());
            telemetry.update();
        }
    }
}