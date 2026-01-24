package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Configuration factory for creating a Swerve-based Follower.
 * Handles the initialization of SwerveDrivetrain and SwerveLocalizer.
 */

    public class SwerveConstants {

        // --- Robot Geometry ---
        public static final double TRACK_WIDTH = 17.258;
        public static final double WHEELBASE = 13.544;

        // --- Swerve Drivetrain Constants ---
        public static final boolean REVERSE_LEFT_FRONT = false;
        public static final boolean REVERSE_RIGHT_FRONT = false;
        public static final boolean REVERSE_LEFT_BACK = true;
        public static final boolean REVERSE_RIGHT_BACK = false;
<<<<<<< HEAD

        public static final double FRONT_LEFT_OFFSET  = 2.74;
        public static final double FRONT_RIGHT_OFFSET = 4.63;
        public static final double BACK_LEFT_OFFSET   = 6.2;
        public static final double BACK_RIGHT_OFFSET  = 2.737;
=======
/*
        public static final double FRONT_LEFT_OFFSET  = 3.6709;
        public static final double FRONT_RIGHT_OFFSET = 4.2173;
        public static final double BACK_LEFT_OFFSET   = 0.8435;
        public static final double BACK_RIGHT_OFFSET  = 3.2509;
>>>>>>> 2db9c0ed4381448cbc3abe2a5de2790c2428c9ed

 */
/*
        public static final double FRONT_LEFT_OFFSET  = 185 /180*3.1415;
        public static final double FRONT_RIGHT_OFFSET = 320 /180*3.1415;
        public static  final double BACK_LEFT_OFFSET   = 225 /180*3.1415;
        public static final double BACK_RIGHT_OFFSET  = 185 /180*3.1415;
*/
public static final double FRONT_LEFT_OFFSET  = 2.74;
    public static final double FRONT_RIGHT_OFFSET = 4.63;
    public static final double BACK_LEFT_OFFSET   = 6.2;
    public static final double BACK_RIGHT_OFFSET  = 2.737;
    //final double FRONT_LEFT_OFFSET  = 5.2417; // This value is in radians (0 to 2π)
    //final double FRONT_RIGHT_OFFSET = 5.7881;
    //final double BACK_LEFT_OFFSET   = 2.4143;
    //final double BACK_RIGHT_OFFSET  = 4.8209;


    // --- Steering PID Constants ---
        public static final double STEER_KP = 0.6;
        public static final double STEER_DEADBAND = 0.05;


    // --- Pinpoint Localizer Offsets (in mm) ---
        // These define where the pods are relative to the robot's center.
        // Forward Pod: Measures X movement.
        public static final double FORWARD_POD_OFFSET_X = -84.0; // Distance forward/back from center (mm)
        public static final double FORWARD_POD_OFFSET_Y = 0.0;   // Distance left/right from center (mm) - usually 0 if centered

        // Strafe Pod: Measures Y movement.
        public static final double STRAFE_POD_OFFSET_X = 0.0;    // Distance forward/back from center (mm)
        public static final double STRAFE_POD_OFFSET_Y = -168.0; // Distance left/right from center (mm)

        // --- Follower PID Constants ---
        // Start conservative. Increase P if sluggish, Decrease if oscillating.
        private static final double TRANS_P = 0.3, TRANS_I = 0.0, TRANS_D = 0.01;
        private static final double HEAD_P = 2.0, HEAD_I = 0.0, HEAD_D = 0.1;
        private static final double DRIVE_P = 0.025, DRIVE_I = 0.0, DRIVE_D = 0.001;

        /**
         * Factory method to create a Swerve-compatible Follower.
         * @param hardwareMap The OpMode's hardware map.
         * @return A fully initialized Follower instance.
         */
        public static Follower createFollower(HardwareMap hardwareMap) {
            // 1. Setup Pedro Pathing Tuning Constants
            FollowerConstants constants = new FollowerConstants();
            constants.coefficientsTranslationalPIDF.setCoefficients(TRANS_P, TRANS_I, TRANS_D, 0);
            constants.coefficientsHeadingPIDF.setCoefficients(HEAD_P, HEAD_I, HEAD_D, 0);
            constants.coefficientsDrivePIDF.setCoefficients(DRIVE_P, DRIVE_I, DRIVE_D, 0.25, 0);

            constants.mass = 16.2; // Adjusted to match your example constants
            constants.centripetalScaling = 0.0005;

            // 2. Initialize Custom Hardware Components
            SwerveLocalizer localizer = new SwerveLocalizer(hardwareMap);
            SwerveDrivetrain drivetrain = new SwerveDrivetrain(hardwareMap);

            // 3. Construct the Follower
            return new Follower(constants, localizer, drivetrain);
        }
    }


