package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp(name = "NebulaArcadeDrive", group = "Drive")
public class NebulaArcadeDrive extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive;
    private DcMotor frontRightDrive, backRightDrive;

    private static final double DEADBAND = 0.05;

    @Override
    public void runOpMode() {

        /* ===== Hardware Map ===== */
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");

        /* ===== Motor Configuration ===== */
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Nebula Arcade Drive Ready");
        telemetry.addLine("Left stick: forward/back + turn");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /* ===== Arcade Inputs ===== */
            double forward = -gamepad1.left_stick_y; // forward/back
            double turn    =  gamepad1.left_stick_x; // rotation

            if (Math.abs(forward) < DEADBAND) forward = 0;
            if (Math.abs(turn) < DEADBAND)    turn = 0;

            /* ===== Arcade Math ===== */
            double leftPower  = forward + turn;
            double rightPower = forward - turn;

            /* ===== Normalize ===== */
            double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            if (max > 1.0) {
                leftPower  /= max;
                rightPower /= max;
            }

            /* ===== Apply Power ===== */
            frontLeftDrive.setPower(leftPower);
            backLeftDrive.setPower(leftPower);
            frontRightDrive.setPower(rightPower);
            backRightDrive.setPower(rightPower);

            telemetry.update();
        }
    }
}
