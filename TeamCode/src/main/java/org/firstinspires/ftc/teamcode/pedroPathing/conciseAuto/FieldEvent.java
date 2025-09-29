package org.firstinspires.ftc.teamcode.pedroPathing.conciseAuto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.pedropathing.util.Timer;


public class FieldEvent {

    // You can store your hardware here, as you'll need it to perform actions
    private static HardwareMap hardwareMap;
   private static RevBlinkinLedDriver blinkinLedDriver;
   private static RevBlinkinLedDriver.BlinkinPattern pattern;
    private static Timer timer;
    private static double duration;



    public static void initialize(HardwareMap hwMap) {
        hardwareMap = hwMap;
        // You can initialize subsystems here if needed, e.g.,
        // Intake.initialize(hardwareMap);

        //pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;

    }

    public static boolean perform(Event event) {
        switch (event) {
            case SHOOT:
                // Logic to shoot a scoring element
                // Example:
                // Shooter.shoot();
                return true;
            case CYCLE:
                // Logic to cycle a scoring element
                // Example:
                // Intake.intake();
                // Shooter.load();
                return true;
            case READ:
                // Logic to read a sensor (e.g., a color sensor or vision)
                // Example:
                // double sensorValue = ColorSensor.getRawValue();
                return true;
            case BLINK:
                // Logic to blink an LED
                // Example:
                // Blinkin.blink();

                //next blinkinpattern

                //pattern = pattern.next();
                return true;
            case PAUSE:
                if (timer.getElapsedTimeSeconds() > duration) {
                    return true; // Pause is complete
                }
                return false; // Pause is still ongoing

            case CALIBRATE:
                // Logic to calibrate a sensor or mechanism
                // Example:
                // Gyro.calibrate();
                //blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);

                return true;
            case NULL:
            default:
                // Do nothing
                return true;
        }
    }
}