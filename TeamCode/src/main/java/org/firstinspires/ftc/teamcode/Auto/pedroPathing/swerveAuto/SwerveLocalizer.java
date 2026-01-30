
package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import static org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.SwerveConstants.*;

/**
 * Localizer implementation for the goBILDA Pinpoint Odometry Computer.
 * Reads position and heading directly from the Pinpoint device.
 */
public class SwerveLocalizer implements Localizer {

    private GoBildaPinpointDriver odo;
    private Pose currentPose = new Pose();
    private Vector velocityVector = new Vector();
    private Pose startPose = new Pose();
    private double totalHeading = 0;
    private double previousHeading = 0;

    public SwerveLocalizer(HardwareMap hw) {
        // Initialize the Pinpoint driver from the hardware map.
        // Ensure your config name matches "odo" or change it here.
        odo = hw.get(GoBildaPinpointDriver.class, "odo");

        // --- PINPOINT CONFIGURATION ---
        // Use constants from SwerveConstants.java
        odo.setOffsets(FORWARD_POD_OFFSET_X, STRAFE_POD_OFFSET_Y,DistanceUnit.MM);

        // Set encoder directions
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Use FORWARD/REVERSED based on your specific wiring

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset position
        odo.resetPosAndIMU();

        // Initialize heading tracking
        previousHeading = 0;
        totalHeading = 0;
    }

    @Override
    public void update() {
        // 1. Bulk read data from the Pinpoint
        odo.update();

        // 2. Get the current pose from the Pinpoint (in Inches and Radians)
        Pose2D pinPose = odo.getPosition();
       // Pose2D pinVel = odo.getVelX(DistanceUnit.INCH);

        // 3. Convert Pinpoint Pose2D to Pedro Pose
        double currentHeading = pinPose.getHeading(AngleUnit.RADIANS);

        currentPose = new Pose(pinPose.getX(DistanceUnit.INCH),pinPose.getY(DistanceUnit.INCH),currentHeading);


        // 4. Update Velocity Vector
        velocityVector.setComponents(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH));

        // 5. Update Total Heading (Accumulated)
        double deltaHeading = currentHeading - previousHeading;

        // Handle wrapping for correct delta calculation
        if (deltaHeading < -Math.PI) {
            deltaHeading += 2 * Math.PI;
        } else if (deltaHeading > Math.PI) {
            deltaHeading -= 2 * Math.PI;
        }

        totalHeading += deltaHeading;
        previousHeading = currentHeading;
    }

    @Override
    public Pose getPose() {
        return currentPose;
    }

    /**
     * Returns the current velocity estimate from the Localizer as a Pose.
     * X and Y represent linear velocity, Heading represents angular velocity.
     */
    @Override
    public Pose getVelocity() {
       // Pose2D pinVel = odo.getVelX();
        return new Pose(
                odo.getVelX(DistanceUnit.INCH),
                odo.getVelY(DistanceUnit.INCH),
                odo.getHeading(AngleUnit.RADIANS)
        );



    }

    @Override
    public Vector getVelocityVector() {
        return velocityVector;
    }

    @Override
    public void setPose(Pose pose) {
        // Send the new pose to the Pinpoint hardware so it tracks correctly
        odo.setPosition(new Pose2D(
                DistanceUnit.INCH,
                pose.getX(),
                pose.getY(),
                AngleUnit.RADIANS,
                pose.getHeading()
        ));
        this.currentPose = pose;

        // Reset heading accumulator to the new pose's heading
        this.previousHeading = pose.getHeading();
        this.totalHeading = pose.getHeading();
    }

    @Override
    public void setStartPose(Pose pose) {
        // Set the initial position on the Pinpoint
        setPose(pose);
        this.startPose = pose;
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    // Required by Interface
    @Override
    public double getForwardMultiplier() {
        return 1.0;
    }

    @Override
    public double getLateralMultiplier() {
        return 1.0;
    }

    @Override
    public double getTurningMultiplier() {
        return 1.0;
    }

    /**
     * Resets the IMU of the localizer.
     * For Pinpoint, this resets the Position and IMU to zero.
     */
    @Override
    public void resetIMU() throws InterruptedException {
        odo.resetPosAndIMU();
        previousHeading = 0;
        totalHeading = 0;
    }

    @Override
    public double getIMUHeading() {
        return currentPose.getHeading();
    }

    @Override
    public boolean isNAN() {
        return Double.isNaN(currentPose.getX()) || Double.isNaN(currentPose.getY());
    }

    // Optional getters (not explicitly in Interface summary but often used by PoseTracker/Follower)
    public double getX() { return currentPose.getX(); }
    public double getY() { return currentPose.getY(); }
    public double getHeading() { return currentPose.getHeading(); }
}