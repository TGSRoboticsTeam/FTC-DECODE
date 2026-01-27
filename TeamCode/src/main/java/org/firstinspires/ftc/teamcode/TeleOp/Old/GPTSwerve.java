package org.firstinspires.ftc.teamcode.TeleOp.Old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@TeleOp(name = "GraciousProfessionalismTerrificSwerve", group = "Swerve")
public class GPTSwerve extends LinearOpMode {

    // --- 1. HARDWARE DECLARATIONS ---
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private CRServo frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;
    private AnalogInput frontLeftEncoder, frontRightEncoder, backLeftEncoder, backRightEncoder;
    private IMU imu;
    private VoltageSensor voltageSensor;
    private DcMotor leftFly, rightFly, intake;
    private Servo trigger;
    private Servo adjuster;
    private Servo light;

    // --- 2. ROBOT GEOMETRY ---
    final double TRACK_WIDTH = 17.258;
    final double WHEELBASE   = 13.544;
    final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // --- 3. CRITICAL: OFFSETS (Using your measured values) ---
    final double FRONT_LEFT_OFFSET  = 5.2417; // This value is in radians (0 to 2π)
    final double FRONT_RIGHT_OFFSET = 5.7881;
    final double BACK_LEFT_OFFSET   = 2.4143;
    final double BACK_RIGHT_OFFSET  = 4.8209;

    // --- 4. TUNING PARAMETERS ---
    final double STEER_KP = 0.6;
    final double DRIVE_DEADBAND = 0.05;
    final double STEER_DEADBAND = 0.05;
    final double ADJUSTER_DEADBAND = 0.05;

    // --- 5. SPEED CONTROL CONSTANTS ---
    final double MAX_SPEED_GLOBAL = 1;
    final double MAX_SPEED_SLOW_MODE = 0.2;
    final double TRIGGER_THRESHOLD = 0.5;
    final double FULL_FLYWHEEL_POWER = 1.0;
    final double SLOW_FLYWHEEL_POWER = 0.75;

    // --- 6. MECHANISM DIRECTION REVERSAL (EASY ACCESS) ---
    final boolean LEFT_FLY_REVERSE    = false;
    final boolean RIGHT_FLY_REVERSE   = true;
    final boolean INTAKE_REVERSE      = true;

    // --- 7. SERVO PARAMETERS ---
    final double SWEEP_DOWN_POSITION = 0.33;
    final double SWEEP_UP_POSITION   = 0.05;
    final long SWEEP_DELAY_MS = 250;

    // Turret Tilt Control Parameters
    final double INITIAL_TURRET_TILT = 0.5;
    final double TURRET_TILT_STEP_STICK = 0.015;
    final double MIN_TURRET_TILT = 0.01;
    final double MAX_TURRET_TILT = 0.99;

    // --- 8. LIGHT SWEEP PARAMETERS (Modified for 3 states) ---
    // Assuming 2.8 -> 0.28 (Default/Off)
    final double LIGHT_DEFAULT_POSITION = 0.28;
    // Keeping 0.5 -> 0.45 (Full Power - from previous code)
    final double LIGHT_FULL_POWER_POSITION = 0.5;
    // Assuming 3.88 -> 0.388 (Partial Power)
    final double LIGHT_PARTIAL_POSITION = 0.35;

    // Power feathering
    final double VOLTAGE_MINIMUM = 12.5;
    final double VOLTAGE_MAXIMUM = 13.6;
    final double FLYWHEEL_MINIMUM = 8.5;
    final double FLYWHEEL_MAXIMUM = 1.0;

    // Flywheel power
    final double FLYWHEEL_DEFAULT_POWER = 0.75;
    final double FLYWHEEL_POWER_STEP = 0.0025;

    // --- 9. TOGGLE STATE VARIABLES ---
    private boolean isCalibrationModeActive = false;
    private boolean rightStickButtonPreviouslyPressed = false;
    private boolean isFlywheelOn = false;
    private boolean isIntakeOn = false;
    private boolean leftTriggerPreviouslyPressed = false;
    private boolean rightTriggerPreviouslyPressed = false;
    private boolean aButtonPreviouslyPressed = false;
    private boolean yButtonPreviouslyPressed = false;
    private boolean isLowPowerMode = false;
    private boolean dpadLeftPreviouslyPressed = false;
    private double flyPower = 0.75;


    @Override
    public void runOpMode() {

        initializeHardware();

        // **CRITICAL**: Reset IMU Yaw on initialization so current direction is 0 degrees.
        imu.resetYaw();

        waitForStart();

        double targetAngleFL = 0, targetAngleFR = 0, targetAngleBL = 0, targetAngleBR = 0;
        double turretTilt = INITIAL_TURRET_TILT;
        double currentFlywheelPower = 0.0;

        // Initialize servos
        trigger.setPosition(SWEEP_DOWN_POSITION);
        adjuster.setPosition(turretTilt);
        // Initialize light to the new default position
        light.setPosition(LIGHT_DEFAULT_POSITION);

        while (opModeIsActive()) {

            // --- INPUT READS ---
            boolean dpadLeft = gamepad1.dpad_left;


            // --- Toggle Logic for Calibration Mode (Gamepad 1 R3) ---
            //  boolean rightStickButtonCurrentlyPressed = gamepad1.right_stick_button;
            // if (rightStickButtonCurrentlyPressed && !rightStickButtonPreviouslyPressed) {
            //     isCalibrationModeActive = !isCalibrationModeActive;
            //   }
            //   rightStickButtonPreviouslyPressed = rightStickButtonCurrentlyPressed;

            // Speed Limiter Logic (Gamepad 1 Right Bumper)
            double speedMultiplier = MAX_SPEED_GLOBAL;
            if (gamepad1.right_bumper) {
                speedMultiplier = MAX_SPEED_SLOW_MODE;
            }

            // --- YAW RESET LOGIC (Gamepad 1 Y Button) ---
            boolean yButtonCurrentlyPressed = gamepad1.y;
            if (yButtonCurrentlyPressed && !yButtonPreviouslyPressed) {
                imu.resetYaw();
                telemetry.addData("Field-Centric", "Yaw Reset to 0 degrees.");
                telemetry.update();
            }
            yButtonPreviouslyPressed = yButtonCurrentlyPressed;

            // Get current heading from IMU (in Radians)`
            double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // --- CALIBRATION MODE CHECK ---
            if (isCalibrationModeActive) {
                runCalibrationMode(
                        new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive},
                        new CRServo[]{frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer}
                );
                // Kill all mechanisms
                leftFly.setPower(0);
                rightFly.setPower(0);
                intake.setPower(0);
                continue;
            }

            // --- SWEEPER SERVO TRIGGER (Gamepad 1 A Button) ---
            boolean aButtonCurrentlyPressed = gamepad1.a;
            if (aButtonCurrentlyPressed && !aButtonPreviouslyPressed) {
                trigger.setPosition(SWEEP_UP_POSITION);
                sleep(SWEEP_DELAY_MS);
                trigger.setPosition(SWEEP_DOWN_POSITION);
            }
            aButtonPreviouslyPressed = aButtonCurrentlyPressed;

            // --- TURRET TILT CONTROL (COMBINED) ---
            double tiltInputStick = -gamepad1.right_stick_y;

            if (Math.abs(tiltInputStick) > ADJUSTER_DEADBAND) {
                turretTilt += tiltInputStick * TURRET_TILT_STEP_STICK;
            }
            adjuster.setPosition(turretTilt);


            boolean flyPowerUpDpad = gamepad1.dpad_up;
            boolean flyPowerDownDpad = gamepad1.dpad_down;

            if (flyPowerUpDpad) {
                flyPower += FLYWHEEL_POWER_STEP;
            } else if (flyPowerDownDpad) {
                flyPower -= FLYWHEEL_POWER_STEP;
            }

            if (flyPower > 1.0) {
                flyPower = 1.0;
            }

            boolean resetFlywheelPower = gamepad1.b;
            if (resetFlywheelPower) {
                flyPower = FLYWHEEL_DEFAULT_POWER;
            }

            if (isFlywheelOn) {
                leftFly.setPower(flyPower);
                rightFly.setPower(flyPower);
            }

            /*
            boolean tiltUpDpad = gamepad1.dpad_up;
            boolean tiltDownDpad = gamepad1.dpad_down;

            if (tiltUpDpad) {
                turretTilt = MAX_TURRET_TILT;
            }
            else if (tiltDownDpad) {
                turretTilt = MIN_TURRET_TILT;
            }

            if (turretTilt < MIN_TURRET_TILT) {
                turretTilt = MIN_TURRET_TILT;
            } else if (turretTilt > MAX_TURRET_TILT) {
                turretTilt = MAX_TURRET_TILT;
            } else if (gamepad1.dpad_right) {
                turretTilt = 0.441;
            }
            adjuster.setPosition(turretTilt);
            //*/

            // --- END TURRET TILT CONTROL ---


            // --- DRIVE INPUTS (FIELD-CENTRIC) ---

            double fieldY = -gamepad1.left_stick_y * speedMultiplier;
            double fieldX = gamepad1.left_stick_x * speedMultiplier;
            double rot = gamepad1.right_stick_x * speedMultiplier;

            double robotX = fieldX * Math.cos(-robotYaw) - fieldY * Math.sin(-robotYaw);
            double robotY = fieldX * Math.sin(-robotYaw) + fieldY * Math.cos(-robotYaw);

            double A = robotX - rot * (WHEELBASE / R);
            double B = robotX + rot * (WHEELBASE / R);
            double C = robotY - rot * (TRACK_WIDTH / R);
            double D = robotY + rot * (TRACK_WIDTH / R);

            double speedFrontLeft  = Math.hypot(B, D);
            double speedFrontRight = Math.hypot(B, C);
            double speedBackLeft   = Math.hypot(A, D);
            double speedBackRight  = Math.hypot(A, C);
            double maxSpeed = Math.max(Math.max(speedFrontLeft, speedFrontRight), Math.max(speedBackLeft, speedBackRight));
            if (maxSpeed > 1.0) {
                speedFrontLeft /= maxSpeed;
                speedFrontRight /= maxSpeed;
                speedBackLeft /= maxSpeed;
                speedBackRight /= maxSpeed;
            }

            if (Math.abs(fieldX) > DRIVE_DEADBAND || Math.abs(fieldY) > DRIVE_DEADBAND || Math.abs(rot) > DRIVE_DEADBAND) {
                targetAngleFL = Math.atan2(B, D);
                targetAngleFR = Math.atan2(B, C);
                targetAngleBL = Math.atan2(A, D);
                targetAngleBR = Math.atan2(A, C);
            } else {
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
            }

            if (gamepad1.left_stick_button) {
                targetAngleFL = Math.PI / 4; targetAngleFR = -Math.PI / 4;
                targetAngleBL = -Math.PI / 4; targetAngleBR = Math.PI / 4;
                speedFrontLeft = 0; speedFrontRight = 0; speedBackLeft = 0; speedBackRight = 0;
            }

            /* Don't drive
            runModule(frontLeftDrive, frontLeftSteer, frontLeftEncoder, FRONT_LEFT_OFFSET, speedFrontLeft, targetAngleFL);
            runModule(frontRightDrive, frontRightSteer, frontRightEncoder, FRONT_RIGHT_OFFSET, speedFrontRight, targetAngleFR);
            runModule(backLeftDrive, backLeftSteer, backLeftEncoder, BACK_LEFT_OFFSET, speedBackLeft, targetAngleBL);
            runModule(backRightDrive, backRightSteer, backRightEncoder, BACK_RIGHT_OFFSET, speedBackRight, targetAngleBR);

             */
            // --- MECHANISM CONTROL ---

            // 1. Flywheel Toggle (Left Trigger)
            boolean leftTriggerCurrentlyPressed = gamepad1.left_trigger > TRIGGER_THRESHOLD;
            if (leftTriggerCurrentlyPressed && !leftTriggerPreviouslyPressed) {
                isFlywheelOn = !isFlywheelOn;
            }
            leftTriggerPreviouslyPressed = leftTriggerCurrentlyPressed;

            // --- POWER ADJUSTMENT TOGGLE (Gamepad 1 D-Pad Left) ---
            if (dpadLeft && !dpadLeftPreviouslyPressed) {
                isLowPowerMode = !isLowPowerMode;
            }
            dpadLeftPreviouslyPressed = dpadLeft;

            // 2. Determine Flywheel Power
            if (isFlywheelOn) {
                if (isLowPowerMode) {
                    currentFlywheelPower = SLOW_FLYWHEEL_POWER;
                } else {
                    currentFlywheelPower = FULL_FLYWHEEL_POWER;
                }
            } else {
                currentFlywheelPower = 0.0;
            }//*/

            /*double voltageMult = FULL_FLYWHEEL_POWER;
            double lightPosition = LIGHT_PARTIAL_POSITION;
            if (isLowPowerMode) {
                voltageMult = SLOW_FLYWHEEL_POWER;
                lightPosition = LIGHT_PARTIAL_POSITION ;
            } else {
                voltageMult = FULL_FLYWHEEL_POWER;
                lightPosition = LIGHT_FULL_POWER_POSITION;
            }


            if (isFlywheelOn) {
                double robotVoltage = voltageSensor.getVoltage();
                double featheredPower = FLYWHEEL_MINIMUM + (robotVoltage - VOLTAGE_MINIMUM) * (FLYWHEEL_MAXIMUM - FLYWHEEL_MINIMUM) / (VOLTAGE_MAXIMUM - VOLTAGE_MINIMUM);
                currentFlywheelPower = Math.max(FLYWHEEL_MINIMUM, Math.min(featheredPower, FLYWHEEL_MAXIMUM)) * voltageMult;


            } else {
                currentFlywheelPower = 0.0;
                lightPosition = LIGHT_DEFAULT_POSITION;
            }*/

            // 3. Apply Flywheel Power
            //leftFly.setPower(currentFlywheelPower);
            //rightFly.setPower(currentFlywheelPower);

            //  light.setPosition(lightPosition);

            // 4. Intake Toggle (Right Trigger)
            boolean rightTriggerCurrentlyPressed = gamepad1.right_trigger > TRIGGER_THRESHOLD;
            if (rightTriggerCurrentlyPressed && !rightTriggerPreviouslyPressed) {
                isIntakeOn = !isIntakeOn;
            }
            rightTriggerPreviouslyPressed = rightTriggerCurrentlyPressed;

            intake.setPower(isIntakeOn ? 1.0 : 0);


            // --- LIGHT SERVO CONTROL (UPDATED 3-STATE LOGIC) ---
            if (currentFlywheelPower >= FULL_FLYWHEEL_POWER) {
                // If at FULL power (1.0), use the full power position (0.45)
                light.setPosition(LIGHT_FULL_POWER_POSITION);
            } else if (currentFlywheelPower > 0.0) {
                // If flywheel is ON but NOT full power (i.e., partial power 0.5), use the partial position (0.388)
                light.setPosition(LIGHT_PARTIAL_POSITION);
            } else {
                // If flywheel is OFF (0.0 power), use the default position (0.28)
                light.setPosition(LIGHT_DEFAULT_POSITION);
            }//*/


            // --- TELEMETRY ---
            telemetry.addData("Mode", "**FIELD-CENTRIC ACTIVE**");
            telemetry.addData("Current Yaw (Degrees)", "%.2f", Math.toDegrees(robotYaw));
            telemetry.addData("Control", "Y Button to Reset Field Forward");
            telemetry.addData("Turret Tilt Position", "%.3f", turretTilt);
            telemetry.addData("Flywheel Power", "%.2f", currentFlywheelPower);
            telemetry.addData("Power Mode", isLowPowerMode ? "LOW (50%)" : "FULL (100%)");
            telemetry.addData("Light Servo Pos", "%.3f", light.getPosition());
            telemetry.addData("Flywheel Power", "%.3f", flyPower);
            telemetry.update();
        }
    }

    // --- HELPER METHODS ---

    private void initializeHardware() {
        // --- Swerve Drive Hardware ---
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");
        frontLeftSteer  = hardwareMap.get(CRServo.class, "frontLeftSteer");
        frontRightSteer = hardwareMap.get(CRServo.class, "frontRightSteer");
        backLeftSteer   = hardwareMap.get(CRServo.class, "backLeftSteer");
        backRightSteer  = hardwareMap.get(CRServo.class, "backRightSteer");
        frontLeftEncoder  = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        backLeftEncoder   = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        backRightEncoder  = hardwareMap.get(AnalogInput.class, "backRightEncoder");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        imu = hardwareMap.get(IMU.class, "imu");

        // --- Mechanism Hardware ---
        leftFly = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");
        intake = hardwareMap.get(DcMotor.class, "intake");
        trigger = hardwareMap.get(Servo.class, "trigger");
        adjuster = hardwareMap.get(Servo.class, "adjuster");
        light = hardwareMap.get(Servo.class, "light");

        // **CRITICAL**: Define the orientation of the Hub on the robot.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(logoDirection, usbDirection)
        );
        imu.initialize(parameters);

        // --- Swerve Drive Motor Direction Fix ---
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // --- Mechanism Motor Direction Fix ---
        leftFly.setDirection(LEFT_FLY_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rightFly.setDirection(RIGHT_FLY_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setDirection(INTAKE_REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // Set Zero Power Behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFly.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset RunMode for all DC motors
        resetMotors(frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, leftFly, rightFly, intake);
    }

    private void runModule(DcMotor driveMotor, CRServo steerServo, AnalogInput encoder, double encoderOffset, double speed, double targetAngle) {
        // Swerve module control logic (PID loop for steering)
        double rawAngle = getRawAngle(encoder);
        double currentAngle = rawAngle - encoderOffset;
        currentAngle = wrapAngle(currentAngle);

        double delta = wrapAngle(targetAngle - currentAngle);

        // Check for 180-degree flip optimization
        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        double servoPower = STEER_KP * delta;
        servoPower *= -1; // Steering Fix: Invert servo power to match physical rotation

        if (Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0;

        servoPower = Math.max(-1, Math.min(1, servoPower));

        steerServo.setPower(servoPower);
        driveMotor.setPower(speed);
    }

    private double wrapAngle(double angle) {
        // Wraps angle between -PI and PI
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private void resetMotors(DcMotor... motors) {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    private double getRawAngle(AnalogInput encoder) {
        // Converts Analog Input voltage (0-3.3V) to Radians (0 to 2π)
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }

    private void runCalibrationMode(DcMotor[] driveMotors, CRServo[] steerServos) {
        // Calibration mode disables power to all motors/servos except for encoder telemetry
        for (DcMotor motor : driveMotors) { motor.setPower(0); }
        for (CRServo servo : steerServos) { servo.setPower(0); }

        telemetry.addData("Mode", "**CALIBRATION - PID DISABLED**");
        telemetry.addData("FL Raw Angle", getRawAngle(frontLeftEncoder));
        telemetry.addData("FR Raw Angle", getRawAngle(frontRightEncoder));
        telemetry.addData("BL Raw Angle", getRawAngle(backLeftEncoder));
        telemetry.addData("BR Raw Angle", getRawAngle(backRightEncoder));
        telemetry.addData("Exit", "Press Right Stick Button (R3) to EXIT.");
        telemetry.update();
    }
}