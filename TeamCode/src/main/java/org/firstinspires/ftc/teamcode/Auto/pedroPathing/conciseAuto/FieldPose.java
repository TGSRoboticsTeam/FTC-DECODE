package org.firstinspires.ftc.teamcode.Auto.pedroPathing.conciseAuto;
// Change package path to match your actual file location

import com.pedropathing.geometry.Pose;
// NOTE: This assumes 'com.pedropathing.geometry.Pose' is the correct package for Pedro Pathing

public class FieldPose {

    // Tile Size Constant
    private static final double TILE_SIZE = 24.0;
    private static final double HALF_TILE = TILE_SIZE / 2.0;
    private static final double HEADING_North = 90/180*3.1415;// All robots facing 0.0 radians
    private static final double HEADING_N = 90/180*3.1415;
    private static final double Heading_NW = 315/180*3.1415;
    private static final double Heading_NE = 45/ 180 * 3.1415;//315 degrees
    private static final double Heading_E = 90/180*3.1415;//270 degrees
    private static final double Heading_W = 270/180*3.1415;

    /**
     * The absolute center of the 6x6 FTC field (144 inches / 2).
     */
    public static final Pose CENTER = new Pose(72.0, 72.0, HEADING_North);

    /**
     * Helper method to calculate the center X-coordinate of a tile column (1=A, 6=F).
     */
    private static double getX(int column) {
        return (column * TILE_SIZE) - HALF_TILE;
    }

    /**
     * Helper method to calculate the center Y-coordinate of a tile row (1=1, 6=6).
     */
    private static double getY(int row) {
        return (row * TILE_SIZE) - HALF_TILE;
    }

    /*
     * Column A (X = 12.0)
     */
    public static final Pose A1 = new Pose(getX(1), getY(1), HEADING_North); // (12.0, 12.0, 0.0)
    public static final Pose A2 = new Pose(getX(1), getY(2), HEADING_North); // (12.0, 36.0, 0.0)
    public static final Pose A3 = new Pose(getX(1), getY(3), HEADING_North); // (12.0, 60.0, 0.0)
    public static final Pose A4 = new Pose(getX(1), getY(4), HEADING_North); // (12.0, 84.0, 0.0)
    public static final Pose A5 = new Pose(getX(1), getY(5), HEADING_North); // (12.0, 108.0, 0.0)
    public static final Pose A6 = new Pose(getX(1), getY(6), HEADING_North); // (12.0, 132.0, 0.0)

    /*
     * Column B (X = 36.0)
     */
    public static final Pose B1 = new Pose(getX(2), getY(1), HEADING_North); // (36.0, 12.0, 0.0)
    public static final Pose B2 = new Pose(getX(2), getY(2), HEADING_North); // (36.0, 36.0, 0.0)
    public static final Pose B3 = new Pose(getX(2), getY(3), HEADING_North); // (36.0, 60.0, 0.0)
    public static final Pose B4 = new Pose(getX(2), getY(4), HEADING_North); // (36.0, 84.0, 0.0)
    public static final Pose B5 = new Pose(getX(2), getY(5), HEADING_North); // (36.0, 108.0, 0.0)
    public static final Pose B6 = new Pose(getX(2), getY(6), HEADING_North); // (36.0, 132.0, 0.0)

    /*
     * Column C (X = 60.0)
     */
    public static final Pose C1 = new Pose(getX(3), getY(1), HEADING_North); // (60.0, 12.0, 0.0)
    public static final Pose C2 = new Pose(getX(3), getY(2), HEADING_North); // (60.0, 36.0, 0.0)
    public static final Pose C3 = new Pose(getX(3), getY(3), HEADING_North); // (60.0, 60.0, 0.0)
    public static final Pose C4 = new Pose(getX(3), getY(4), HEADING_North); // (60.0, 84.0, 0.0)
    public static final Pose C5 = new Pose(getX(3), getY(5), HEADING_North); // (60.0, 108.0, 0.0)
    public static final Pose C6 = new Pose(getX(3), getY(6), HEADING_North); // (60.0, 132.0, 0.0)

    /*
     * Column D (X = 84.0)
     */
    public static final Pose D1 = new Pose(getX(4), getY(1), HEADING_North); // (84.0, 12.0, 0.0)
    public static final Pose D2 = new Pose(getX(4), getY(2), HEADING_North); // (84.0, 36.0, 0.0)
    public static final Pose D3 = new Pose(getX(4), getY(3), HEADING_North); // (84.0, 60.0, 0.0)
    public static final Pose D4 = new Pose(getX(4), getY(4), HEADING_North); // (84.0, 84.0, 0.0)
    public static final Pose D5 = new Pose(getX(4), getY(5), HEADING_North); // (84.0, 108.0, 0.0)
    public static final Pose D6 = new Pose(getX(4), getY(6), HEADING_North); // (84.0, 132.0, 0.0)

    /*
     * Column E (X = 108.0)
     */
    public static final Pose E1 = new Pose(getX(5), getY(1), HEADING_North); // (108.0, 12.0, 0.0)
    public static final Pose E2 = new Pose(getX(5), getY(2), HEADING_North); // (108.0, 36.0, 0.0)
    public static final Pose E3 = new Pose(getX(5), getY(3), HEADING_North); // (108.0, 60.0, 0.0)
    public static final Pose E4 = new Pose(getX(5), getY(4), HEADING_North); // (108.0, 84.0, 0.0)
    public static final Pose E5 = new Pose(getX(5), getY(5), HEADING_North); // (108.0, 108.0, 0.0)
    public static final Pose E6 = new Pose(getX(5), getY(6), HEADING_North); // (108.0, 132.0, 0.0)

    /*
     * Column F (X = 132.0)
     */
    public static final Pose F1 = new Pose(getX(6), getY(1), HEADING_North); // (132.0, 12.0, 0.0)
    public static final Pose F2 = new Pose(getX(6), getY(2), HEADING_North); // (132.0, 36.0, 0.0)
    public static final Pose F3 = new Pose(getX(6), getY(3), HEADING_North); // (132.0, 60.0, 0.0)
    public static final Pose F4 = new Pose(getX(6), getY(4), HEADING_North); // (132.0, 84.0, 0.0)
    public static final Pose F5 = new Pose(getX(6), getY(5), HEADING_North); // (132.0, 108.0, 0.0)
    public static final Pose F6 = new Pose(getX(6), getY(6), HEADING_North); // (132.0, 132.0, 0.0)

    public static final Pose A1r45 = new Pose(getX(1), getY(1), Heading_NE); // (12.0, 12.0, 0.0)

    public static final Pose redDepot = new Pose(getX(6) - 8, getY(6) - 8, Heading_NE); // (12.0, 12.0, 0.0)
    public static final Pose blueDepot = new Pose(16, getY(6) - 8, Heading_NW);
    //public static final Pose redShoot = new Pose(getX(4) , getY(4) , Heading_NE); // (12.0, 12.0, 0.0)
    //public static final Pose redGreen = new Pose(getX(5) , getY(4) , Heading_E); // (12.0, 12.0, 0.0)
    //public static final Pose redBarney = new Pose(getX(4) , getY(3) , Heading_E); // (12.0, 12.0, 0.0)
    //public static final Pose redPurple = new Pose(getX(3) , getY(2) + 12, Heading_E); // (12.0, 12.0, 0.0)
    public static final Pose redFourCorners = new Pose(getX(4) + 12, getY(4) + 12, Heading_NE);
    public static final Pose blueFourCorners = new Pose(getX(2) + 12, getY(4) + 12, Heading_NW);
    public static final Pose Gert = new Pose(getX(5), getY(4), Heading_E);
    public static final Pose Mule = new Pose(getX(2), getY(4) + 12, Heading_W);

    public static final Pose redTruck = new Pose(getX(6) + 3, getY(4), Heading_E);
    public static final Pose blueTruck = new Pose(getX(1) - 3, getY(4), Heading_E);




}
