package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

// Import the static poses from your FieldPose class
import static org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.FieldPose.*;

// NOTE: Change the package above to match your file structure!

public class FieldPaths {

    // Store the Follower instance to use the PathBuilder
    private static Follower follower;

    // =========================================================================
    // 1. Path Chain Declarations
    // =========================================================================

    // Long Edge Paths
    public static PathChain A1F1, F1A1, A1A6, A6A1, A6F6, F6A6, F6F1, F1F6;

    // Corner Diagonal Paths
    public static PathChain A1F6, F6A1, F1A6, A6F1;

    // Center Paths (Only defining A1, A6, F1, F6 to keep example concise.
    // You can fill in the other 32 tiles as needed.)
    public static PathChain A1_CENTER, CENTER_A1;
    public static PathChain A6_CENTER, CENTER_A6;
    public static PathChain F1_CENTER, CENTER_F1;
    public static PathChain F6_CENTER, CENTER_F6;
    public static PathChain A2_CENTER, CENTER_A2;
    public static PathChain F2_CENTER, CENTER_F2;
    public static PathChain A5_CENTER, CENTER_A5;
    public static PathChain F5_CENTER, CENTER_F5;

    //Game Paths Red
    public static PathChain redDepotShoot, redShootBarney, redBarneyShoot, redShootGreen, redGreenShoot, redShootPurple, redPurpleShoot;
    public static PathChain redShootCenter;

    public static PathChain Utah, Arizona, Colorado,NewMexico, Push1R, Push2R, Push3R, NewMexico2, NewMexico3, RoadTrip1, RoadTrip2, RoadTrip3;
            //NewMexicoB,Colorado2;



    // =========================================================================
    // 2. Initialization Method (Must be called from OpMode.init())
    // =========================================================================

    /**
     * Initializes all static PathChain objects using the provided Follower's pathBuilder.
     * This method must be called once during OpMode initialization.
     * @param f The Follower instance from the OpMode.
     */
    public static void initializePaths(Follower f) {
        follower = f;

        // --- Helper Method to Build Straight Line PathChain (Similar to ExampleAuto) ---
        // This maintains the heading while moving in a straight line.
        // It uses constant interpolation (heading does not change)

        // NOTE: The linear interpolation in your example uses start/end heading for BezierLine.
        // For simple point-to-point travel, using the start heading for the entire path
        // is often more stable, or you can use .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
        // as shown in your ExampleAuto. The choice depends on the desired motion.

        A1F1 = buildLineChain(A1, F1, A1.getHeading(), F1.getHeading());
        F1A1 = buildLineChain(F1, A1, F1.getHeading(), A1.getHeading());

        A1A6 = buildLineChain(A1, A6, A1.getHeading(), A6.getHeading());
        A6A1 = buildLineChain(A6, A1, A6.getHeading(), A1.getHeading());

        A6F6 = buildLineChain(A6, F6, A6.getHeading(), F6.getHeading());
        F6A6 = buildLineChain(F6, A6, F6.getHeading(), A6.getHeading());

        F6F1 = buildLineChain(F6, F1, F6.getHeading(), F1.getHeading());
        F1F6 = buildLineChain(F1, F6, F1.getHeading(), F6.getHeading());

        // Corner Diagonal Paths
        A1F6 = buildLineChain(A1, F6, A1.getHeading(), F6.getHeading());
        F6A1 = buildLineChain(F6, A1, F6.getHeading(), A1.getHeading());

        F1A6 = buildLineChain(F1, A6, F1.getHeading(), A6.getHeading());
        A6F1 = buildLineChain(A6, F1, A6.getHeading(), F1.getHeading());

        // Center Paths
        A1_CENTER = buildLineChain(A1, CENTER, A1.getHeading(), CENTER.getHeading());
        CENTER_A1 = buildLineChain(CENTER, A1, CENTER.getHeading(), A1.getHeading());

        A6_CENTER = buildLineChain(A6, CENTER, A6.getHeading(), CENTER.getHeading());
        CENTER_A6 = buildLineChain(CENTER, A6, CENTER.getHeading(), A6.getHeading());

        F1_CENTER = buildLineChain(F1, CENTER, F1.getHeading(), CENTER.getHeading());
        CENTER_F1 = buildLineChain(CENTER, F1, CENTER.getHeading(), F1.getHeading());

        F6_CENTER = buildLineChain(F6, CENTER, F6.getHeading(), CENTER.getHeading());
        CENTER_F6 = buildLineChain(CENTER, F6, CENTER.getHeading(), F6.getHeading());

        A2_CENTER = buildLineChain(A2, CENTER, A2.getHeading(), CENTER.getHeading());
        CENTER_A2 = buildLineChain(CENTER, A2, CENTER.getHeading(), A2.getHeading());

        F2_CENTER = buildLineChain(F2, CENTER, F2.getHeading(), CENTER.getHeading());
        CENTER_F2 = buildLineChain(CENTER, F2, CENTER.getHeading(), F2.getHeading());

        A5_CENTER = buildLineChain(A5, CENTER, A5.getHeading(), CENTER.getHeading());
        CENTER_A5 = buildLineChain(CENTER, A5, CENTER.getHeading(), A5.getHeading());

        F5_CENTER = buildLineChain(F5, CENTER, F5.getHeading(), CENTER.getHeading());
        CENTER_F5 = buildLineChain(CENTER, F5, CENTER.getHeading(), F5.getHeading());

        //GamePaths Red
       /** redDepotShoot = buildLineChain(redDepot, redShoot, redDepot.getHeading(), redShoot.getHeading());
        redShootBarney = buildLineChain(redShoot, redBarney, redShoot.getHeading(), redBarney.getHeading());
        redBarneyShoot = buildLineChain(redBarney, redShoot, redBarney.getHeading(), redShoot.getHeading());
        redShootGreen = buildLineChain(redShoot, redGreen, redShoot.getHeading(), redGreen.getHeading());
        redGreenShoot = buildLineChain(redGreen, redShoot, redGreen.getHeading(), redShoot.getHeading());
        redShootPurple = buildLineChain(redShoot, redPurple, redShoot.getHeading(), redPurple.getHeading());
        redPurpleShoot = buildLineChain(redPurple, redShoot, redPurple.getHeading(), redShoot.getHeading());
        redShootCenter = buildLineChain(redShoot, CENTER, redShoot.getHeading(), CENTER.getHeading());
        // NOTE: Add the remaining 32 tiles to/from CENTER here.*/

       Colorado = buildLineChain(redDepot, redFourCorners, redDepot.getHeading(), redFourCorners.getHeading());
       NewMexico = buildLineChain(redFourCorners, P1R, redFourCorners.getHeading(), P1R.getHeading());
       Push1R = buildLineChain(P1R, F1R, P1R.getHeading(), F1R.getHeading());
       RoadTrip1 = buildLineChain(F1R, redFourCorners, F1R.getHeading(), redFourCorners.getHeading());
       NewMexico2 = buildLineChain(redFourCorners, P2R, redFourCorners.getHeading(), P1R.getHeading());
       Push2R = buildLineChain(P2R, F2R, P2R.getHeading(), F2R.getHeading());
       RoadTrip2 = buildLineChain(F2R, redFourCorners, F2R.getHeading(), redFourCorners.getHeading());
       NewMexico3 = buildLineChain(redFourCorners, P3R, redFourCorners.getHeading(), P3R.getHeading());
       Push3R = buildLineChain(P3R, F3R, P3R.getHeading(), F3R.getHeading());
       RoadTrip3 = buildLineChain(F3R, redFourCorners, F3R.getHeading(), redFourCorners.getHeading());




        /*
        NewMexicoB = buildLineChain(Gert, GertB, Gert.getHeading(), GertB.getHeading());
        Colorado2 = buildLineChain(GertB, redFourCorners, GertB.getHeading(), redFourCorners.getHeading());
       */


        Utah = buildLineChain(blueDepot, blueFourCorners, blueDepot.getHeading(), blueFourCorners.getHeading());
       Arizona = buildLineChain(blueFourCorners, Mule, blueFourCorners.getHeading(), Mule.getHeading());
    }

    /**
     * Internal helper to build a PathChain that is a single BezierLine segment.
     */
    private static PathChain buildLineChain(Pose start, Pose end, double startHeading, double endHeading) {
        if (follower == null) {
            throw new IllegalStateException("FieldPaths.initializePaths must be called before accessing paths.");
        }
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(startHeading, endHeading)
                .build();

    }
}
