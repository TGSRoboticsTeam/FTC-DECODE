package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
//@Disabled
@TeleOp(name = "Camera Target Indicator", group = "Concept")
public class CameraTrials extends LinearOpMode {
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    final int RED_TAG = 24;
    final int BLUE_TAG = 20;
    private int current_tag_id = RED_TAG;

    @Override
    public void runOpMode() {
        initAprilTag();

        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }
            sleep(20);

            double horizontalOffset = getXOffsetToTagInches(current_tag_id);
            double distance = getZOffsetToTagInches(current_tag_id);

            double angleToTag = Math.atan(horizontalOffset/distance);

            telemetry.addData("Target Tag ID", current_tag_id);
            telemetry.addData("Goal Dist (inches)", distance);
            telemetry.addData("Angle To Tag", angleToTag);
            telemetry.update();
        }
        visionPortal.close();
    }
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private double getXOffsetToTagInches(int desiredId) {
        if (aprilTag == null) return Double.NaN;

        AprilTagDetection best = null;
        for (AprilTagDetection det : aprilTag.getDetections()) {
            if (det == null) continue;
            if (det.id != desiredId) continue;
            if (det.ftcPose == null) continue;

            double r = det.ftcPose.x;
            if (Double.isNaN(r) || Double.isInfinite(r) || r <= 0) continue;

            if (best == null || r < best.ftcPose.x) best = det;
        }

        if (best == null) return Double.NaN;
        return best.ftcPose.x;
    }

    private double getYOffsetToTagInches(int desiredId) {
        if (aprilTag == null) return Double.NaN;

        AprilTagDetection best = null;
        for (AprilTagDetection det : aprilTag.getDetections()) {
            if (det == null) continue;
            if (det.id != desiredId) continue;
            if (det.ftcPose == null) continue;

            double r = det.ftcPose.y;
            if (Double.isNaN(r) || Double.isInfinite(r) || r <= 0) continue;

            if (best == null || r < best.ftcPose.y) best = det;
        }

        if (best == null) return Double.NaN;
        return best.ftcPose.y;
    }

    private double getZOffsetToTagInches(int desiredId) {
        if (aprilTag == null) return Double.NaN;

        AprilTagDetection best = null;
        for (AprilTagDetection det : aprilTag.getDetections()) {
            if (det == null) continue;
            if (det.id != desiredId) continue;
            if (det.ftcPose == null) continue;

            double r = det.ftcPose.z;
            if (Double.isNaN(r) || Double.isInfinite(r) || r <= 0) continue;

            if (best == null || r < best.ftcPose.z) best = det;
        }

        if (best == null) return Double.NaN;
        return best.ftcPose.z;
    }

}