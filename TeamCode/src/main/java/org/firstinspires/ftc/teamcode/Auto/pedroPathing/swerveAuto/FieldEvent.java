package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;



public class FieldEvent {

    // You can store your hardware here, as you'll need it to perform actions
    private static HardwareMap hardwareMap;
   private static RevBlinkinLedDriver blinkinLedDriver;
   private static RevBlinkinLedDriver.BlinkinPattern pattern;
    private static Timer timerEvent;
    private static double duration;
    static DcMotor leftFlywheel;
    static DcMotor rightFlywheel;
    static DcMotor intake;

    static final boolean INTAKE_REVERSE = true;




    public static void initialize(HardwareMap hwMap) {
        hardwareMap = hwMap;
        leftFlywheel = hardwareMap.get(DcMotor.class, "leftFly");
        rightFlywheel = hardwareMap.get(DcMotor.class, "rightFly");

        // Intake --- NEW ---
        intake = hardwareMap.get(DcMotor.class, "intake");
        // Set Direction based on TeleOp logic
        intake.setDirection(INTAKE_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // You can initialize subsystems here if needed, e.g.,
        // Intake.initialize(hardwareMap);

        //pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;

    }

    public static boolean perform(Event event) {
        switch (event) {
            case INTAKE_ON:
                // Turn intake on full power
                intake.setPower(1.0);
                return true; // Immediate action, return true instantly

            case INTAKE_OFF:
                // Turn intake off
                intake.setPower(0.0);
                return true; // Immediate action, return true instantly

            case SHOOT:
                // Logic to shoot a scoring element
                leftFlywheel.setPower(1.0);
                rightFlywheel.setPower(1.0);
                pauseTime(5);
                //leftFlap.setPosition(0.25);
                //rightFlap.setPosition(0.75);
                pauseTime(1);
              //  leftFlap.setPosition(0);
              //  rightFlap.setPosition(1);

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

                //pattern = pattern.next()
                return true;
            case PAUSE:
                if (timerEvent.getElapsedTimeSeconds() > duration) {
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
    public static void pauseTime(double t) {
        timerEvent.resetTimer();
        while (timerEvent.getElapsedTimeSeconds() < t) {


        }
    }

}