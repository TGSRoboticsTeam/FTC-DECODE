package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Helper class to calculate Field-Centric Pose from Robot-Centric AprilTag detections.
 */
public class AprilTagPose {

    /**
     * Calculates the robot's Field-Centric Pose based on an AprilTag detection.
     * * @param detection The detection object from the AprilTag processor.
     * @param tagFieldPose The known location and orientation of the AprilTag on the field.
     * Heading should be the direction the tag is FACING (e.g., pointing towards the field center).
     * @return A Pose representing the robot's position and heading on the field.
     */
    public static Pose calculateRobotPose(AprilTagDetection detection, Pose tagFieldPose) {
        // 1. Get Robot-Centric Data from Detection
        // range: distance to tag (hypotenuse)
        // bearing: angle from camera center to tag (yaw)
        // yaw: angle of the tag relative to the camera (orientation)

        double range = detection.ftcPose.range;
        double bearing = Math.toRadians(detection.ftcPose.bearing);
        double yaw = Math.toRadians(detection.ftcPose.yaw);

        // 2. Calculate Robot's Field Heading
        // The tag has a known heading. The camera sees the tag at a certain 'yaw'.
        // If tag is facing West (180 deg) and camera sees it flat (0 yaw), robot is facing East (0 deg).
        // Formula depends on coordinate system, but generally:
        double robotHeading = tagFieldPose.getHeading() - Math.PI + yaw; // Normalize as needed

        // 3. Calculate Robot's Position relative to the Tag (Field Frame)
        // We know the distance (range) and the angle from the tag to the robot.
        // Angle from tag to robot = TagHeading + (Equation derived from bearing/yaw geometry)

        // Simplified approach: Calculate relative offsets in rotated frame
        double relativeX = range * Math.cos(bearing); // Forward/Back distance to tag
        double relativeY = range * Math.sin(bearing); // Strafe distance to tag

        // Rotate these relative offsets by the Robot's Heading to get Field-Centric offsets
        double fieldXOffset = relativeX * Math.cos(robotHeading) - relativeY * Math.sin(robotHeading);
        double fieldYOffset = relativeX * Math.sin(robotHeading) + relativeY * Math.cos(robotHeading);

        // 4. Determine Robot Field Position
        // The calculation above finds the tag relative to the robot.
        // To find robot relative to tag, we subtract the offsets.
        // Note: Camera offset from robot center should also be subtracted here for full accuracy.

        // Simple approximation (assuming camera is center of robot):
        // This math effectively "backs up" from the tag's pose to find the robot.
        // It requires precise trigonometry based on the specific axis conventions of the SDK.

        // Alternative simple strategy for distance/angle (User requested):
        // Just returning the raw distance/angle is "Robot Centric".
        // Returning the Pose below is "Field Centric".

        // Ideally, use the tag's pose and project backward along the viewing angle.
        double tagToRobotAngle = tagFieldPose.getHeading() + yaw; // Vector pointing out of tag

        double robotX = tagFieldPose.getX() - (range * Math.cos(tagToRobotAngle - bearing));
        double robotY = tagFieldPose.getY() - (range * Math.sin(tagToRobotAngle - bearing));

        return new Pose(robotX, robotY, robotHeading);
    }
}