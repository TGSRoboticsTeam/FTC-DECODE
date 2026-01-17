package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import static org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.SwerveConstants.*;

public class SwerveDrivetrain extends Drivetrain {

    private DcMotor flDrive, frDrive, blDrive, brDrive;
    private CRServo flSteer, frSteer, blSteer, brSteer;
    private VoltageSensor voltageSensor;

    // Derived Geometry
    private final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

    // Internal State
    private double xVel = 0, yVel = 0;
    private double[] targetAngles = new double[4]; // Stores calculated steering angles

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

        voltageSensor = hw.voltageSensor.iterator().next();

        // Configure Motors
        flDrive.setDirection(REVERSE_LEFT_FRONT ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        frDrive.setDirection(REVERSE_RIGHT_FRONT ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        blDrive.setDirection(REVERSE_LEFT_BACK ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        brDrive.setDirection(REVERSE_RIGHT_BACK ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.nominalVoltage = 12.0;
        this.maxPowerScaling = 1.0;
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

        double x = combined.getXComponent();
        double y = combined.getYComponent();

        // 3. Determine Rotation Power
        double rot = headingPower.getMagnitude() * Math.signum(MathFunctions.getTurnDirection(robotHeading, headingPower.getTheta()));

        // 4. Swerve Kinematics
        double A = x - rot * (WHEELBASE / R);
        double B = x + rot * (WHEELBASE / R);
        double C = y - rot * (TRACK_WIDTH / R);
        double D = y + rot * (TRACK_WIDTH / R);

        // 5. Calculate Speeds
        double flP = Math.hypot(B, D);
        double frP = Math.hypot(B, C);
        double blP = Math.hypot(A, D);
        double brP = Math.hypot(A, C);

        // 6. Calculate Angles (Radians) and store for runDrive
        targetAngles[0] = Math.atan2(B, D);
        targetAngles[1] = Math.atan2(B, C);
        targetAngles[2] = Math.atan2(A, D);
        targetAngles[3] = Math.atan2(A, C);

        return new double[]{flP, frP, blP, brP};
    }

    /**
     * Executes the calculated drive powers.
     */
    @Override
    public void runDrive(double[] drivePowers) {
        // Voltage Compensation
        double multiplier = voltageCompensation ? (nominalVoltage / getVoltage()) : 1.0;
        multiplier *= maxPowerScaling;

        // Set Drive Powers
        flDrive.setPower(drivePowers[0] * multiplier);
        frDrive.setPower(drivePowers[1] * multiplier);
        blDrive.setPower(drivePowers[2] * multiplier);
        brDrive.setPower(drivePowers[3] * multiplier);

        // --- STEERING LOGIC ---
        // CURRENT STATUS: ENCODERS OFFLINE -> FIXED MODE
        // Once cables are fixed, implement PID here using targetAngles[]
        flSteer.setPower(0);
        frSteer.setPower(0);
        blSteer.setPower(0);
        brSteer.setPower(0);
    }

    @Override
    public void updateConstants() { /* No dynamic constants to update */ }

    @Override
    public void breakFollowing() {
        flDrive.setPower(0); frDrive.setPower(0);
        blDrive.setPower(0); brDrive.setPower(0);
        flSteer.setPower(0); frSteer.setPower(0);
        blSteer.setPower(0); brSteer.setPower(0);
    }

    @Override
    public void startTeleopDrive() { startTeleopDrive(true); }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        setZeroPowerBehavior(brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // Velocity getters required by interface but unused in Holonomic pathing
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
        return "Swerve Drive: [Mode: FIXED (Encoders Offline)]";
    }
}