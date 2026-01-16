package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.pedropathing.util.Timer;

public class FieldEvent {

    // Hardware
    private static HardwareMap hardwareMap;
    static DcMotor leftFlywheel;
    static DcMotor rightFlywheel;
    static DcMotor intake;

    // Constants
    static final boolean INTAKE_REVERSE = true;

    // Timing Logic
    private static Timer timerEvent = new Timer(); // Initialize here
    private static boolean isTimerRunning = false; // Flag to track if we started the timer for an event

    public static void initialize(HardwareMap hwMap) {
        hardwareMap = hwMap;
        leftFlywheel = hardwareMap.get(DcMotor.class, "leftFly");
        rightFlywheel = hardwareMap.get(DcMotor.class, "rightFly");

        // Intake
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(INTAKE_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public static boolean perform(Event event) {
        switch (event) {
            case INTAKE_ON:
                intake.setPower(1.0);
                return true;

            case INTAKE_OFF:
                intake.setPower(0.0);
                return true;

            case SHOOT:
                // Non-blocking approach for shooting
                // 1. Start Flywheels
                leftFlywheel.setPower(1.0);
                rightFlywheel.setPower(1.0);

                // 2. Wait for spin up (Example: 1 second)
                if (!isTimerRunning) {
                    timerEvent.resetTimer();
                    isTimerRunning = true;
                }

                if (timerEvent.getElapsedTimeSeconds() > 1.0) {
                    // 3. Stop (or actuate feeder) after duration
                    leftFlywheel.setPower(0);
                    rightFlywheel.setPower(0);
                    isTimerRunning = false; // Reset flag for next event
                    return true; // Event complete
                }
                return false; // Event still running

            case PAUSE:
                // Example: Pause for 2 seconds
                if (!isTimerRunning) {
                    timerEvent.resetTimer();
                    isTimerRunning = true;
                }

                if (timerEvent.getElapsedTimeSeconds() > 2.0) {
                    isTimerRunning = false;
                    return true;
                }
                return false;
            case BLINK:
               // telemetry.addData("Status", "BLINK");
                return true;

            case CALIBRATE:
                return true;

            case NULL:
            default:
                return true;
        }
    }

    // Blocking pause (Use only if you absolutely must stop the whole thread)
    public static void pauseTime(double t) {
        Timer tempTimer = new Timer();
        tempTimer.resetTimer();
        while (tempTimer.getElapsedTimeSeconds() < t) {
            // Wait
        }
    }
}