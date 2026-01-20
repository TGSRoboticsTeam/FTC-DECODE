package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
//@Disabled
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

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("frontLeftEncoder", frontLeftEncoder.getVoltage());
            telemetry.addData("frontRightEncoder", frontRightEncoder.getVoltage());
            telemetry.addData("backLeftEncoder", backLeftEncoder.getVoltage());
            telemetry.addData("backRightEncoder", backRightEncoder.getVoltage());
            telemetry.update();
        }
    }
}