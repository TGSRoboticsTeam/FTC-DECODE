package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "OutreachBot", group = "TeleOp")
public class OutreachBot extends LinearOpMode {

    // Deadzone helper
    private double applyDeadzone(double v, double dz) {
        return (Math.abs(v) < dz) ? 0.0 : v;
    }

    @Override
    public void runOpMode() {

        // ----------------- DRIVE MOTORS (ROBOT-CENTRIC MECANUM) -----------------
        DcMotor leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ----------------- FLYWHEELS -----------------
        DcMotor leftFlywheel  = hardwareMap.get(DcMotor.class, "left_fly");
        DcMotor rightFlywheel = hardwareMap.get(DcMotor.class, "right_fly");

        leftFlywheel.setDirection(DcMotor.Direction.FORWARD);
        rightFlywheel.setDirection(DcMotor.Direction.REVERSE);

        // ----------------- INTAKE -----------------
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);

        // ----------------- SERVOS -----------------
        Servo leftFlap     = hardwareMap.get(Servo.class, "left_flap");
        Servo rightFlap    = hardwareMap.get(Servo.class, "right_flap");
        Servo turretServo  = hardwareMap.get(Servo.class, "turret_servo");
        Servo trigger      = hardwareMap.get(Servo.class, "trigger");

        // IMU not used for robot-centric drive, but safe to initialize
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(null));

        // ----------------- CONSTANTS -----------------
        final double BASE_DRIVE_SCALE = 0.50;   // 🔥 50% speed always
        final double SLOW_MODE_SCALE  = 0.50;   // Right bumper = additional slow

        final double DZ_XY  = 0.06;
        final double DZ_ROT = 0.08;

        final double TRIGGER_UP   = 0.0;
        final double TRIGGER_DOWN = 0.15;

        final int LOAD_BALL_TICKS = 50;

        // ----------------- STATE -----------------
        boolean flywheelsOn = false;
        boolean intakeOn = false;

        boolean prevA = false;
        boolean prevLT = false;
        boolean prevRT = false;

        boolean triggerSequencing = false;
        long triggerStartTimeMs = 0;

        int flapTimer = 0;

        double turretTilt = 0.0;
        final double turretStep = 0.02;

        // Initial positions
        trigger.setPosition(TRIGGER_DOWN);
        leftFlap.setPosition(0.0);
        rightFlap.setPosition(1.0);
        turretServo.setPosition(turretTilt);

        telemetry.addLine("OutreachBot READY");
        telemetry.addLine("Drive speed limited to 50%");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ----------------- ROBOT-CENTRIC DRIVE -----------------
            double y   = -gamepad1.left_stick_y;
            double x   =  gamepad1.left_stick_x;
            double rot =  gamepad1.right_stick_x;

            y   = applyDeadzone(y, DZ_XY);
            x   = applyDeadzone(x, DZ_XY);
            rot = applyDeadzone(rot, DZ_ROT);

            // Always 50% speed
            y   *= BASE_DRIVE_SCALE;
            x   *= BASE_DRIVE_SCALE;
            rot *= BASE_DRIVE_SCALE;

            // Optional extra slow (right bumper)
            if (gamepad1.right_bumper) {
                y   *= SLOW_MODE_SCALE;
                x   *= SLOW_MODE_SCALE;
                rot *= SLOW_MODE_SCALE;
            }

            double fl = y + x + rot;
            double bl = y - x + rot;
            double fr = y - x - rot;
            double br = y + x - rot;

            double max = Math.max(1.0,
                    Math.max(Math.abs(fl),
                            Math.max(Math.abs(bl),
                                    Math.max(Math.abs(fr), Math.abs(br)))));

            fl /= max; bl /= max; fr /= max; br /= max;

            if (y == 0 && x == 0 && rot == 0) {
                fl = bl = fr = br = 0;
            }

            leftFrontDrive.setPower(fl);
            leftBackDrive.setPower(bl);
            rightFrontDrive.setPower(fr);
            rightBackDrive.setPower(br);

            // ----------------- TRIGGER SEQUENCE (A) -----------------
            boolean aNow = gamepad1.a;
            if (aNow && !prevA && !triggerSequencing) {
                triggerSequencing = true;
                triggerStartTimeMs = System.currentTimeMillis();
                trigger.setPosition(TRIGGER_UP);
            }
            prevA = aNow;

            if (triggerSequencing) {
                if (System.currentTimeMillis() - triggerStartTimeMs >= 1000) {
                    trigger.setPosition(TRIGGER_DOWN);
                    triggerSequencing = false;
                }
            }

            // ----------------- FLYWHEEL TOGGLE (LT) -----------------
            boolean ltNow = gamepad1.left_trigger > 0.2;
            if (ltNow && !prevLT) flywheelsOn = !flywheelsOn;
            prevLT = ltNow;

            // ----------------- INTAKE TOGGLE (RT) -----------------
            boolean rtNow = gamepad1.right_trigger > 0.2;
            if (rtNow && !prevRT) intakeOn = !intakeOn;
            prevRT = rtNow;

            leftFlywheel.setPower(flywheelsOn ? 1.0 : 0.0);
            rightFlywheel.setPower(flywheelsOn ? 1.0 : 0.0);
            intake.setPower(intakeOn ? 1.0 : 0.0);

            // ----------------- FLAP (B) -----------------
            if (gamepad1.b) flapTimer = LOAD_BALL_TICKS;

            if (flapTimer > 0) {
                flapTimer--;
                leftFlap.setPosition(0.25);
                rightFlap.setPosition(0.75);
            } else {
                leftFlap.setPosition(0.0);
                rightFlap.setPosition(1.0);
            }

            // ----------------- TURRET TILT -----------------
            if (gamepad1.dpad_up)   turretTilt += turretStep;
            if (gamepad1.dpad_down) turretTilt -= turretStep;
            turretTilt = Math.max(0.0, Math.min(1.0, turretTilt));
            turretServo.setPosition(turretTilt);

            // ----------------- TELEMETRY -----------------
            telemetry.addData("Drive Scale", gamepad1.right_bumper ? "25%" : "50%");
            telemetry.addData("Trigger Seq", triggerSequencing);
            telemetry.addData("Flywheels", flywheelsOn);
            telemetry.addData("Intake", intakeOn);
            telemetry.update();
        }
    }
}
