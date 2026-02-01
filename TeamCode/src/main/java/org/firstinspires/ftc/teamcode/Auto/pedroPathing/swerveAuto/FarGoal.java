package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Simplified FAR Red", group = "Swerve")
public class FarGoal extends LinearOpMode {

    // --- Hardware ---
    private DcMotor flDrive, frDrive, blDrive, brDrive;
    private CRServo flSteer, frSteer, blSteer, brSteer;
    private AnalogInput flEnc, frEnc, blEnc, brEnc;
    private DcMotor leftFly, rightFly, frontIntake, backIntake;
    private Servo pusher;
    private Servo adjuster;
    private GoBildaPinpointDriver odo;

    // --- Swerve Constants ---
    private final double FL_OFFSET = 2.74, FR_OFFSET = 4.63, BL_OFFSET = 6.2, BR_OFFSET = 2.737;
    private final double STEER_KP = 0.6;

    /* ===================== TRIGGER SERVO ===================== */
    // Your mapping: 0.0 = launched, 0.225 = down/reset
    final double TRIGGER_FIRE = 0.0;
    final double TRIGGER_HOME = 0.225;

    final long TRIGGER_PULSE_MS = 250;

    final long LAUNCH_TRIGGER_HOLD_MS = 400;
    final long TRIGGER_RESET_WAIT_MS = 150;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addData("Status", "Initialized. Resetting Odo...");
        telemetry.update();

        waitForStart();
        sleep(10000);

        // 1. Launch Start: Firing 3 times
        adjuster.setPosition(.118);
        turnOnFlys(0.78);

        //turnAllPods(1.5); 45 degrees in some direction

        sleep(1000);
        fireOne();
        frontIntake.setPower(0.75);
        sleep(1200);
        frontIntake.setPower(0);
        fireOne();
        backIntake.setPower(-0.75);
        sleep(1200);
        backIntake.setPower(0);
        fireOne();

        turnOnFlys(0);
        setDrive(0.4, 0.4, -0.4, -0.4);
        sleep(1500);
        setDrive(0.0, 0.0, 0.0, 0.0);
        sleep(25000);



        // 2. Align Pods Left/Right & Pause 1s
        turnAllPods(Math.toRadians(90));
        sleep(1000);

        // 3. Move 51 inches Right
        driveRightDistance(51);
        sleep(500);

        // 4. Intake Cycle
        turnFrontIntakeOn(1.0);
        sleep(500);
        turnAllPods(Math.toRadians(0)); // Align Forward/Back
        sleep(1000);

        // 5. Move Forward 30 inches
        driveForwardDistance(30);
        sleep(500);

        // 6. Turn off Intakes & Return
        turnFrontIntakeOn(0);
        driveBackwardDistance(30);

        turnAllPods(Math.toRadians(90));
        sleep(1000);
        driveLeftDistance(51);
        sleep(1000);

        // 7. Launch Start firing 3 times
        turnOnFlys(1.0);
        sleep(1000);
        for(int i=0; i<3; i++) { fireOne(); }
        turnOnFlys(0);
        sleep(1000);

        // 8. Diagonal Move 50 inches at 45 degrees
        turnAllPods(Math.toRadians(45));
        sleep(1000);
        driveDiagonalDistance(50);

        telemetry.addData("Status", "Auto Complete");
        telemetry.update();
    }
    public void fireOne() {
        pusher.setPosition(TRIGGER_FIRE);
        sleep(800);
        pusher.setPosition(TRIGGER_HOME);
        sleep(800);
    }

    // --- Odometry Drive Methods ---

    public void driveForwardDistance(double inches) {
        double startX = getX();
        while (opModeIsActive() && (getX() - startX) < inches) {
            setDrive(0.4, 0.4, 0.4, 0.4);
            updateOdoTelemetry();
        }
        stopDrivetrain();
    }

    public void driveBackwardDistance(double inches) {
        double startX = getX();
        while (opModeIsActive() && (startX - getX()) < inches) {
            setDrive(-0.4, -0.4, -0.4, -0.4);
            updateOdoTelemetry();
        }
        stopDrivetrain();
    }

    public void driveRightDistance(double inches) {
        double startY = getY();
        // In standard field coords, moving right decreases Y
        while (opModeIsActive() && (startY - getY()) < inches) {
            setDrive(0.4, -0.4, 0.4, -0.4);
            updateOdoTelemetry();
        }
        stopDrivetrain();
    }

    public void driveLeftDistance(double inches) {
        double startY = getY();
        while (opModeIsActive() && (getY() - startY) < inches) {
            setDrive(-0.4, 0.4, -0.4, 0.4);
            updateOdoTelemetry();
        }
        stopDrivetrain();
    }

    public void driveDiagonalDistance(double inches) {
        double startX = getX();
        double startY = getY();
        double target = inches;
        while (opModeIsActive()) {
            double currentDist = Math.hypot(getX() - startX, getY() - startY);
            if (currentDist >= target) break;
            setDrive(0.4, 0.4, 0.4, 0.4);
            updateOdoTelemetry();
        }
        stopDrivetrain();
    }

    // --- Helper Methods ---

    public void turnAllPods(double target) {
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - startTime < 1000) {
            updatePod(flSteer, flEnc, FL_OFFSET, target);
            updatePod(frSteer, frEnc, FR_OFFSET, target);
            updatePod(blSteer, blEnc, BL_OFFSET, target);
            updatePod(brSteer, brEnc, BR_OFFSET, target);
        }
        stopSteer();
    }

    private void updatePod(CRServo steer, AnalogInput enc, double offset, double target) {
        //double current = wrap((enc.getVoltage() / 3.3 * (2 * Math.PI)) - offset);
        double current = wrap( (2 * Math.PI) - offset);

        double error = wrap(target - current);
        steer.setPower(-STEER_KP * error);
    }

    private double getX() { odo.update(); return odo.getPosition().getX(DistanceUnit.INCH); }
    private double getY() { odo.update(); return odo.getPosition().getY(DistanceUnit.INCH); }

    public void setDrive(double fl, double fr, double bl, double br) {
        flDrive.setPower(fl); frDrive.setPower(fr);
        blDrive.setPower(bl); brDrive.setPower(br);
    }

    public void stopDrivetrain() { setDrive(0,0,0,0); }
    public void stopSteer() { flSteer.setPower(0); frSteer.setPower(0); blSteer.setPower(0); brSteer.setPower(0); }

    public void turnOnFlys(double p) { leftFly.setPower(p*-1); rightFly.setPower(p); }



    public void turnFrontIntakeOn(double p) { frontIntake.setPower(p); }

    private void initHardware() {
        // Drive Motors
        flDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        blDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        brDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        // Steer Servos and Encoders
        flSteer  = hardwareMap.get(CRServo.class, "frontLeftSteer");
        frSteer = hardwareMap.get(CRServo.class, "frontRightSteer");
        blSteer   = hardwareMap.get(CRServo.class, "backLeftSteer");
        brSteer  = hardwareMap.get(CRServo.class, "backRightSteer");

        flEnc  = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frEnc = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        blEnc   = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        brEnc  = hardwareMap.get(AnalogInput.class, "backRightEncoder");




        // Mechanism Motors
        leftFly = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        pusher = hardwareMap.get(Servo.class, "trigger");
        adjuster = hardwareMap.get(Servo.class, "adjuster");


        // Pinpoint Odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-15.0, -147.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
    }

    private void updateOdoTelemetry() {
        odo.update();
        telemetry.addData("X", getX());
        telemetry.addData("Y", getY());
        telemetry.update();
    }

    private double wrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}