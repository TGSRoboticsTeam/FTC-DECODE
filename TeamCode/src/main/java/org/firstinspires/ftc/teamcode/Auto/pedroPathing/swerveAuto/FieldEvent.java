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
    static DcMotor frontIntake;
    static DcMotor backIntake;

    private static int shotCount = 0;
    private static final double SPIN_UP_TIME = 1.0;

    // New variables to track the 3-shot cycle
    private static int shotCounter = 0;
    private static Timer timerEvent = new Timer();
    private static boolean isTimerRunning = false;

    // Constants
    static final boolean INTAKE_REVERSE = true;


    public static void initialize(HardwareMap hwMap) {
        hardwareMap = hwMap;
        leftFlywheel = hardwareMap.get(DcMotor.class, "leftFly");
        rightFlywheel = hardwareMap.get(DcMotor.class, "rightFly");

        // Intake
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        frontIntake.setDirection(INTAKE_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        backIntake.setDirection(INTAKE_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        backIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public static boolean perform(Event event) {
        switch (event) {
            case LAUNCH_START_THREE:
                return handleTripleLaunch();

            case INTAKE_ON:
                frontIntake.setPower(1.0);
                backIntake.setPower(1.0);
                return true;

            case INTAKE_OFF:
                frontIntake.setPower(0.0);
                backIntake.setPower(0.0);
                return true;

            case ALIGN_LEFT_RIGHT:
                // Routine to rotate pods 90 degrees without driving
                return true;

            case ALIGN_FORWARD_BACK:
                // Routine to rotate pods 0 degrees without driving
                return true;

            case ALIGN_DIAGONAL:
                // Routine to rotate pods 45 degrees without driving
                return true;

            case PAUSE_1SEC:
                return handleWait(1.0);

            case PAUSE_05SEC:
                return handleWait(0.5);

            default:
                return true;
        }
    }

    /**
     * Logic: Turns on flywheels, waits for spin-up, increments count,
     * and repeats until 3 shots are "fired."
     */
    private static boolean handleTripleLaunch() {
        if (shotCounter >= 3) {
            shotCounter = 0; // Reset for the next time we need to fire
            leftFlywheel.setPower(0);
            rightFlywheel.setPower(0);
            return true; // Tells ComplexPathAuto to move to the next step
        }

        // Start flywheels
        leftFlywheel.setPower(1.0);
        rightFlywheel.setPower(1.0);

        if (!isTimerRunning) {
            timerEvent.resetTimer();
            isTimerRunning = true;
        }

        // Wait 1.5 seconds per "shot" to simulate spin-up and firing
        if (timerEvent.getElapsedTimeSeconds() > 1.5) {
            shotCounter++;
            isTimerRunning = false; // Reset timer for the next shot in the cycle
        }

        return false; // Tells ComplexPathAuto we are still busy firing
    }

    private static boolean handleWait(double seconds) {
        if (!isTimerRunning) {
            timerEvent.resetTimer();
            isTimerRunning = true;
        }
        if (timerEvent.getElapsedTimeSeconds() > seconds) {
            isTimerRunning = false;
            return true;
        }
        return false;
    }
}
