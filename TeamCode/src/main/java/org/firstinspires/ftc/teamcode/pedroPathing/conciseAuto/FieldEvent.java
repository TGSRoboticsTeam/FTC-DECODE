package org.firstinspires.ftc.teamcode.pedroPathing.conciseAuto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;


public class FieldEvent {

    // You can store your hardware here, as you'll need it to perform actions
    private static HardwareMap hardwareMap;
    private static RevBlinkinLedDriver blinkinLedDriver;
    private static RevBlinkinLedDriver.BlinkinPattern pattern;
    private static Timer timer;
    private static double duration;
    private static DcMotor leftFlywheel;
    private static DcMotor rightFlywheel;
    private static Servo leftFlap;
    private static Servo rightFlap;



    public static void initialize(HardwareMap hwMap) {
        hardwareMap = hwMap;
        leftFlywheel = hardwareMap.get(DcMotor.class, "left_fly");
        rightFlywheel = hardwareMap.get(DcMotor.class, "right_fly");
        leftFlap = hardwareMap.get(Servo.class, "left_flap");
        rightFlap = hardwareMap.get(Servo.class, "right_flap");
        // You can initialize subsystems here if needed, e.g.,
        // Intake.initialize(hardwareMap);


        //pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;

    }

    public static void pauseTime(double t){
        timer.resetTimer();
        while(timer.getElapsedTimeSeconds() < t){
        }
    }

    public static boolean perform(Event event) {
        switch (event) {
            case SHOOT3:

                leftFlywheel.setPower(1);
                rightFlywheel.setPower(1);
                pauseTime(5);
                leftFlap.setPosition(0.25);
                rightFlap.setPosition(0.75);
                pauseTime(.5);
                leftFlap.setPosition(0);
                rightFlap.setPosition(1);
                pauseTime(5);
                leftFlap.setPosition(0.25);
                rightFlap.setPosition(0.75);
                pauseTime(.5);
                leftFlap.setPosition(0);
                rightFlap.setPosition(1);
                pauseTime(5);
                leftFlap.setPosition(0.25);
                rightFlap.setPosition(0.75);
                pauseTime(.5);
                leftFlap.setPosition(0);
                rightFlap.setPosition(1);
                pauseTime(5);
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