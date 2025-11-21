package org.firstinspires.ftc.teamcode.Auto.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.function.Supplier;
@TeleOp(name = "QuickTele", group = "Linear OpMode")
public class QuickTele extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Call this once per loop
        DcMotor testMotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        testMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while (!isStarted()) {

        }

        double right_trigger = gamepad1.right_trigger;
        double left_trigger = gamepad1.left_trigger;

        testMotor.setPower(right_trigger - left_trigger);
    }
}