package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "TriggerTest", group = "Tools")
public class TriggerTest extends LinearOpMode {

    private Servo trigger;

    // Trigger positions
    final double TRIGGER_ZERO = 0.0;
    final double TRIGGER_DOWN = 0.2;
    final double TRIGGER_UP   = 0.25;

    private double currentTriggerPosition = TRIGGER_DOWN;

    @Override
    public void runOpMode() {

        trigger = hardwareMap.get(Servo.class, "trigger");

        // Start in safe position
        trigger.setPosition(currentTriggerPosition);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                currentTriggerPosition = TRIGGER_UP;
            }

            if (gamepad1.b) {
                currentTriggerPosition = TRIGGER_DOWN;
            }

            if (gamepad1.x) {
                currentTriggerPosition = TRIGGER_ZERO;
            }

            trigger.setPosition(currentTriggerPosition);

            telemetry.addData("Trigger Position", "%.3f", currentTriggerPosition);
            telemetry.update();
        }
    }
}
