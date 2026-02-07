package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name = "Pose-Based Manual Tracker", group = "Testing")
public class PoseManualTracker extends OpMode {
    private Follower follower;
    private PathChain squarePath;

    private AnalogInput flEnc,frEnc,blEnc,brEnc;
    private final double VOLTAGE_TO_RAD = (2 * Math.PI) / 3.3;

    @Override
    public void init() {
        follower = SwerveConstants.createFollower(hardwareMap);
        HardwareMap  hw = hardwareMap;
        flEnc = hw.get(AnalogInput.class, "frontLeftEncoder");
        frEnc = hw.get(AnalogInput.class, "frontRightEncoder");
        blEnc = hw.get(AnalogInput.class, "backLeftEncoder");
        brEnc = hw.get(AnalogInput.class, "backRightEncoder");

        // Define Poses: Pose(double x, double y, double headingInRadians)
        // This creates a 24x24 square where the robot always faces "forward" (0 rad)
        Pose startPose = new Pose(0, 0, 0);
        Pose point1 = new Pose(24, 0, 0);
        Pose point2 = new Pose(24, 24, 0);
        Pose point3 = new Pose(0, 24, 0);

        // Building the PathChain using Poses
        // .addPath(new Path(new BezierPoint(new Point(startPose)))) 
        // creates a linear path between those specific poses.
        squarePath = follower.pathBuilder()
                // Path from (0,0) to (24,0)
                .addPath(new Path(new BezierLine(startPose,point1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), point1.getHeading())

                // Path from (24,0) to (24,24)
                .addPath(new Path(new BezierLine(point1,point2)))
                .setLinearHeadingInterpolation(point1.getHeading(), point2.getHeading())

                // Path from (24,24) to (0,24)
                .addPath(new Path(new BezierLine(point2,point3)))
                .setLinearHeadingInterpolation(point2.getHeading(), point3.getHeading())

                // Path back to (0,0)
                .addPath(new Path(new BezierLine(point3,startPose)))
                .setLinearHeadingInterpolation(point3.getHeading(), startPose.getHeading())
                .build();

        follower.followPath(squarePath);
    }

    @Override
    public void loop() {
        follower.update();

        Pose currentPose = follower.getPose();
        // Extracting target from the current segment's end point
        Pose targetPose = follower.getCurrentPath().getLastControlPoint().getPose();

        double fle = flEnc.getVoltage() * VOLTAGE_TO_RAD;
        double ble = blEnc.getVoltage() * VOLTAGE_TO_RAD;
        double bre = brEnc.getVoltage() * VOLTAGE_TO_RAD;
        double fre = frEnc.getVoltage() * VOLTAGE_TO_RAD;

        // Telemetry for Panels
        telemetry.addData("X", String.format("%.2f", currentPose.getX()));
        telemetry.addData("Y", String.format("%.2f", currentPose.getY()));
        telemetry.addData("Heading (Deg)", Math.toDegrees(currentPose.getHeading()));

        telemetry.addLine("--- Targets ---");
        telemetry.addData("Target X", targetPose.getX());
        telemetry.addData("Target Y", targetPose.getY());

        telemetry.addLine("--- Swerve Pod ---");
        telemetry.addData("Encoder Rads FL: ", String.format("%.4f", fle));
        telemetry.addData("Encoder Rads FR: ", String.format("%.4f", fre));
        telemetry.addData("Encoder Rads BL :", String.format("%.4f", ble));
        telemetry.addData("Encoder Rads BR: ", String.format("%.4f", bre));





        telemetry.update();
    }
}