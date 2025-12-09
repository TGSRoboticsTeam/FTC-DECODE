package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

    @TeleOp(name="FieldCentric Swerve", group="Swerve")
    public class GraciousProfessionalismTestSwerve extends LinearOpMode { //G.P.T.Swerve, Your welcome.

        // -----------------------------
        // Hardware
        // -----------------------------
        private DcMotor motorA, motorB, motorC, motorD;
        private CRServo servoA, servoB, servoC, servoD;

        private AnalogInput podA, podB, podC, podD;

        private IMU imu;

        // -----------------------------
        // Geometry (in inches)
        // -----------------------------
        final double TRACK_WIDTH = 17.258;
        final double WHEELBASE   = 13.544;

        final double R = Math.hypot(TRACK_WIDTH, WHEELBASE);

        // -----------------------------
        // Auto-align X target (in inches)
        // -----------------------------
        double targetX = 0;

        @Override
        public void runOpMode() {

            // -----------------------------
            // Map hardware
            // -----------------------------
            motorA = hardwareMap.get(DcMotor.class, "motorA");
            motorB = hardwareMap.get(DcMotor.class, "motorB");
            motorC = hardwareMap.get(DcMotor.class, "motorC");
            motorD = hardwareMap.get(DcMotor.class, "motorD");

            servoA = hardwareMap.get(CRServo.class, "servoA");
            servoB = hardwareMap.get(CRServo.class, "servoB");
            servoC = hardwareMap.get(CRServo.class, "servoC");
            servoD = hardwareMap.get(CRServo.class, "servoD");

            podA = hardwareMap.get(AnalogInput.class, "podA");
            podB = hardwareMap.get(AnalogInput.class, "podB");
            podC = hardwareMap.get(AnalogInput.class, "podC");
            podD = hardwareMap.get(AnalogInput.class, "podD");

            imu = hardwareMap.get(IMU.class, "imu");

            IMU.Parameters myIMUparameters;
            myIMUparameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

            resetMotors(motorA, motorB, motorC, motorD);

            waitForStart();

            double headingOffset = 0;

            while (opModeIsActive()) {

                //------------------------------------------------------------------
                // ZERO HEADING
                //------------------------------------------------------------------
                if (gamepad1.start) {
                    headingOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                }

                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - headingOffset;

                //------------------------------------------------------------------
                // Driver Input
                //------------------------------------------------------------------
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rot = gamepad1.right_stick_x;

                // Field-centric transform
                double cosA = Math.cos(heading);
                double sinA = Math.sin(heading);

                double robotX = x * cosA - y * sinA;
                double robotY = x * sinA + y * cosA;

                //------------------------------------------------------------------
                // AUTO ALIGN TO X POSITION
                //------------------------------------------------------------------
                if (gamepad1.left_bumper) {
                    double error = targetX - robotX;     // fast proportional adjust
                    robotX = error * 0.5;
                }

                //------------------------------------------------------------------
                // Swerve math
                //------------------------------------------------------------------
                double A = robotX - rot * (WHEELBASE / R);
                double B = robotX + rot * (WHEELBASE / R);
                double C = robotY - rot * (TRACK_WIDTH / R);
                double D = robotY + rot * (TRACK_WIDTH / R);

                // wheel speeds
                double speedA = Math.hypot(B, D);
                double speedB = Math.hypot(B, C);
                double speedC = Math.hypot(A, C);
                double speedD = Math.hypot(A, D);

                // wheel angles
                double angleA = Math.atan2(B, D);
                double angleB = Math.atan2(B, C);
                double angleC = Math.atan2(A, C);
                double angleD = Math.atan2(A, D);

                //------------------------------------------------------------------
                // Apply to each module
                //------------------------------------------------------------------
                runModule(motorA, servoA, podA, speedA, angleA);
                runModule(motorB, servoB, podB, speedB, angleB);
                runModule(motorC, servoC, podC, speedC, angleC);
                runModule(motorD, servoD, podD, speedD, angleD);

                //------------------------------------------------------------------
                // Telemetry
                //------------------------------------------------------------------
                telemetry.addData("Heading (deg)", Math.toDegrees(heading));
                telemetry.addData("Target X", targetX);
                telemetry.addData("Stick X", x);
                telemetry.update();
            }
        }

        // ------------------------------------------------------------
        // Steering + optimization (CR servo)
        // ------------------------------------------------------------
        private void runModule(DcMotor motor, CRServo servo, AnalogInput encoder, double speed, double targetAngle) {

            double current = encoder.getVoltage() / 3.3 * (2 * Math.PI);

            double delta = wrapAngle(targetAngle - current);

            // 180° optimization
            if (Math.abs(delta) > Math.PI / 2) {
                delta = wrapAngle(delta + Math.PI);
                speed *= -1;
            }

            // Steering PID (simple P only — stable & fast)
            double kP = 1.1;
            double servoPower = kP * delta;

            // deadband to prevent hunting
            if (Math.abs(servoPower) < 0.02) servoPower = 0;

            servo.setPower(servoPower);

            // wheel motor
            motor.setPower(speed);
        }

        // Angle wrapping to [-pi, pi]
        private double wrapAngle(double a) {
            while (a > Math.PI) a -= 2 * Math.PI;
            while (a < -Math.PI) a += 2 * Math.PI;
            return a;
        }

        private void resetMotors(DcMotor... motors) {
            for (DcMotor m : motors) {
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }
