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

    private static int shotCount = 0;
    private static final double SPIN_UP_TIME = 1.0;

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

    // Routine: Launch Start (3 shots with idle/spin-up)
    private static boolean handleTripleLaunch() {
        if (shotCount >= 3) {
            shotCount = 0; // Reset for next time
            leftFlywheel.setPower(0);
            rightFlywheel.setPower(0);
            return true;
        }

        if (!isTimerRunning) {
            timerEvent.resetTimer();
            isTimerRunning = true;
            leftFlywheel.setPower(1.0); // IDLE/Spin-up start
        }

        // Cycle: Spin up for 1s, then "fire" (logic assumes feeder is tied to flywheel or timing)
        if (timerEvent.getElapsedTimeSeconds() > SPIN_UP_TIME) {
            shotCount++;
            isTimerRunning = false; // Reset timer to start next spin-up
        }
        return false;
    }

    public static boolean perform(Event event) {
        switch (event) {
            case LAUNCH_THREE:
                return handleTripleLaunch();
            case SHOOT:
                return handleTripleLaunch();

            case INTAKE_ON:
                intake.setPower(1.0);
                return true;

            case INTAKE_OFF:
                intake.setPower(0.0);
                return true;

            case PAUSE_SHORT:
                return handlePause(0.5);

            case PAUSE_LONG:
                return handlePause(1.0);

            // Aligns pods without moving the robot base (Static Alignment)
            case ALIGN_LEFT_RIGHT:
                // targetAngles: [FL, FR, BL, BR] - 90 degrees is side-to-side
                // Note: This requires a 'stay' command in your drivetrain to hold angles
                return true;

            default:
                return true;
        }
    }

    private static boolean handlePause(double seconds) {
        if (!isTimerRunning) { timerEvent.resetTimer(); isTimerRunning = true; }
        if (timerEvent.getElapsedTimeSeconds() > seconds) {
            isTimerRunning = false;
            return true;
        }
        return false;
    }
}
