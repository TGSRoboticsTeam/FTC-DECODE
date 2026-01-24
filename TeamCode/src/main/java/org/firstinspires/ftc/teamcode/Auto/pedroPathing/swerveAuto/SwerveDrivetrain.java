package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import static org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.SwerveConstants.*;

public class SwerveDrivetrain extends Drivetrain {

    private DcMotor flDrive, frDrive, blDrive, brDrive;
    private CRServo flSteer, frSteer, blSteer, brSteer;
    private AnalogInput flEnc, frEnc, blEnc, brEnc;
    private VoltageSensor voltageSensor;

    // Derived Geometry
    private final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // Internal State
    private double xVel = 0, yVel = 0;
    // Stores calculated steering angles: [FL, FR, BL, BR]
    private double[] targetAngles = new double[4];

    public SwerveDrivetrain(HardwareMap hw) {
        // Initialize Hardware
        flDrive = hw.get(DcMotor.class, "frontLeftDrive");
        frDrive = hw.get(DcMotor.class, "frontRightDrive");
        blDrive = hw.get(DcMotor.class, "backLeftDrive");
        brDrive = hw.get(DcMotor.class, "backRightDrive");

        flSteer = hw.get(CRServo.class, "frontLeftSteer");
        frSteer = hw.get(CRServo.class, "frontRightSteer");
        blSteer = hw.get(CRServo.class, "backLeftSteer");
        brSteer = hw.get(CRServo.class, "backRightSteer");

        // Initialize Analog Encoders
        flEnc = hw.get(AnalogInput.class, "frontLeftEncoder");
        frEnc = hw.get(AnalogInput.class, "frontRightEncoder");
        blEnc = hw.get(AnalogInput.class, "backLeftEncoder");
        brEnc = hw.get(AnalogInput.class, "backRightEncoder");

        voltageSensor = hw.voltageSensor.iterator().next();

        // Configure Motors
        flDrive.setDirection(REVERSE_LEFT_FRONT ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        frDrive.setDirection(REVERSE_RIGHT_FRONT ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        blDrive.setDirection(REVERSE_LEFT_BACK ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        brDrive.setDirection(REVERSE_RIGHT_BACK ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.nominalVoltage = 12.0;
        this.maxPowerScaling = 0.5;

    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        flDrive.setZeroPowerBehavior(behavior);
        frDrive.setZeroPowerBehavior(behavior);
        blDrive.setZeroPowerBehavior(behavior);
        brDrive.setZeroPowerBehavior(behavior);
    }

    /**
     * Calculates drive powers and steering angles from Pedro vectors.
     */
    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // 1. Combine Translational Vectors
        Vector combined = correctivePower.plus(pathingPower);

        // 2. Rotate to Robot-Centric (Pedro vectors are usually Field-Centric)
        combined.rotateVector(-robotHeading);

         //Division by 2 added to slow this thing down
        double x = combined.getXComponent()/2;
        double y = combined.getYComponent()/2;

        // 3. Determine Rotation Power
        double rot = headingPower.getMagnitude() * Math.signum(MathFunctions.getTurnDirection(robotHeading, headingPower.getTheta()));

        // 4. Swerve Kinematics
        double A = x - rot * (WHEELBASE / R);
        double B = x + rot * (WHEELBASE / R);
        double C = y - rot * (TRACK_WIDTH / R);
        double D = y + rot * (TRACK_WIDTH / R);

        // 5. Calculate Speeds
        double flP = Math.hypot(B, D)/2;
        double frP = Math.hypot(B, C)/2;
        double blP = Math.hypot(A, D)/2;
        double brP = Math.hypot(A, C)/2;

        // 5.5 Normalize wheel speeds (CRITICAL)
        double max = Math.max(
                Math.max(flP, frP),
                Math.max(blP, brP)
        );
        if (max > 1.0) {
            flP /= max;
            frP /= max;
            blP /= max;
            brP /= max;
        }

        // 6. Calculate Angles (Radians) and store for runDrive
        targetAngles[0] = Math.atan2(B, D);
        targetAngles[1] = Math.atan2(B, C);
        targetAngles[2] = Math.atan2(A, D);
        targetAngles[3] = Math.atan2(A, C);

        return new double[]{flP, frP, blP, brP};
    }

    /**
     * Executes the calculated drive powers and steers wheels.
     */
    @Override
    public void runDrive(double[] drivePowers) {
        // Voltage Compensation
        double multiplier = voltageCompensation ? (nominalVoltage / getVoltage()) : 1.0;
        multiplier *= maxPowerScaling;

        // Apply control to each module
        runModule(flDrive, flSteer, flEnc, FRONT_LEFT_OFFSET, drivePowers[0] * multiplier, targetAngles[0]);
        runModule(frDrive, frSteer, frEnc, FRONT_RIGHT_OFFSET, drivePowers[1] * multiplier, targetAngles[1]);
        runModule(blDrive, blSteer, blEnc, BACK_LEFT_OFFSET, drivePowers[2] * multiplier, targetAngles[2]);
        runModule(brDrive, brSteer, brEnc, BACK_RIGHT_OFFSET, drivePowers[3] * multiplier, targetAngles[3]);
    }

    /**
     * PID Control Logic for a single Swerve Module (Copied/Adapted from TeleOp)
     */
    private void runModule(DcMotor driveMotor, CRServo steerServo, AnalogInput encoder, double offset, double speed, double targetAngle) {
        // 1. Calculate Current Angle
        double rawAngle = getRawAngle(encoder);
        double currentAngle = wrapAngle(rawAngle - offset);

        // 2. Calculate Error (Delta)
        double delta = wrapAngle(targetAngle - currentAngle);

        // 3. Optimization (Flip 180 if closer)
        if (Math.abs(delta) > Math.PI / 2) {
            delta = wrapAngle(delta + Math.PI);
            speed *= -1;
        }

        // 4. Calculate Steering Power (P-Controller)
        double servoPower = STEER_KP * delta;
        servoPower *= -1; // Invert for physical gear direction

        // Deadband
        if (Math.abs(servoPower) < STEER_DEADBAND) servoPower = 0;

        // Clamp
        servoPower = Math.max(-1, Math.min(1, servoPower));

        // 5. Apply
        steerServo.setPower(servoPower);
        speed = Math.max(-maxPowerScaling, Math.min(maxPowerScaling, speed));
        driveMotor.setPower(speed);
        ;
    }

    private double getRawAngle(AnalogInput encoder) {
        // Convert 0-3.3V to 0-2PI Radians
        return encoder.getVoltage() / 3.3 * (2 * Math.PI);
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    @Override
    public void updateConstants() { /* No dynamic constants to update */ }

    @Override
    public void breakFollowing() {
        // Stop all motors and servos
        flDrive.setPower(0); frDrive.setPower(0);
        blDrive.setPower(0); brDrive.setPower(0);
        flSteer.setPower(0); frSteer.setPower(0);
        blSteer.setPower(0); brSteer.setPower(0);
    }

    @Override
    public void startTeleopDrive() { startTeleopDrive(true); }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        setZeroPowerBehavior(brakeMode ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override public double xVelocity() { return xVel; }
    @Override public double yVelocity() { return yVel; }
    @Override public void setXVelocity(double x) { this.xVel = x; }
    @Override public void setYVelocity(double y) { this.yVel = y; }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    @Override
    public String debugString() {
        // --- UPDATED DEBUG STRING FOR ANGLE DIAGNOSTICS ---
        // Displays:
        // 1. Raw Angle (from Servo Encoder)
        // 2. Calculated Angle (Raw - Offset)
        // 3. Target Angle (from Pedro math)
        return String.format(
                "FL: Raw:%.2f | Calc:%.2f | Tgt:%.2f\n" +
                        "FR: Raw:%.2f | Calc:%.2f | Tgt:%.2f\n" +
                        "BL: Raw:%.2f | Calc:%.2f | Tgt:%.2f\n" +
                        "BR: Raw:%.2f | Calc:%.2f | Tgt:%.2f",
                getRawAngle(flEnc), wrapAngle(getRawAngle(flEnc) - FRONT_LEFT_OFFSET), targetAngles[0],
                getRawAngle(frEnc), wrapAngle(getRawAngle(frEnc) - FRONT_RIGHT_OFFSET), targetAngles[1],
                getRawAngle(blEnc), wrapAngle(getRawAngle(blEnc) - BACK_LEFT_OFFSET), targetAngles[2],
                getRawAngle(brEnc), wrapAngle(getRawAngle(brEnc) - BACK_RIGHT_OFFSET), targetAngles[3]
        );
    }

    public void stopDriveMotorsOnly() {
    }
}