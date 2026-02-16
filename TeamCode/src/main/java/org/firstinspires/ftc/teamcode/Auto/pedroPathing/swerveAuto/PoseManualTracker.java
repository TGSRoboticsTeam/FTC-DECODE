package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.bylazar.telemetry.PanelsTelemetry;

@Config
@Disabled
@TeleOp(name = "Cartesian Manual Tracker", group = "Testing")
public class PoseManualTracker extends OpMode {
    private Follower follower;
    private FtcDashboard dashboard;
    private PanelsTelemetry pt;

    // We use individual Paths instead of a PathChain to prevent blending
    private Path side1, side2, side3, side4,side5,side6;
    private int pathState = 0;

    private AnalogInput flEnc, frEnc, blEnc, brEnc;
    private final double VOLTAGE_TO_RAD = (2 * Math.PI) / 3.3;

    public static double ROBOT_WIDTH = 18.0;
    public static double ROBOT_LENGTH = 18.0;
    public static double TARGET_SQUARE_INCHES = 12.0;
    public static double ARRIVAL_TOLERANCE = 0.5; // How close before switching paths

    private Pose p0,p1,p2,p3,p4,p5,p6;

    //public static final double STEER_KP = 1.0;
    //        public static final double STEER_DEADBAND = 0.10;
    //
     //constants.coefficientsTranslationalPIDF.setCoefficients(TRANS_P, TRANS_I, TRANS_D, 0);
       //     constants.coefficientsHeadingPIDF.setCoefficients(HEAD_P, HEAD_I, HEAD_D, 0);
         //   constants.coefficientsDrivePIDF.setCoefficients(DRIVE_P, DRIVE_I, DRIVE_D, 0.25, 0);


    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        pt = PanelsTelemetry.INSTANCE;
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        follower = SwerveConstants.createFollower(hardwareMap);

        flEnc = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frEnc = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        blEnc = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        brEnc = hardwareMap.get(AnalogInput.class, "backRightEncoder");

        // Define Poses
        p0 = new Pose(0, 0, 0);
        p1 = new Pose(TARGET_SQUARE_INCHES, 0, 0);
        p2 = new Pose(TARGET_SQUARE_INCHES+4, 0, 0);
        p3 = new Pose(TARGET_SQUARE_INCHES+4, TARGET_SQUARE_INCHES, 0);
        p4 = new Pose(TARGET_SQUARE_INCHES, TARGET_SQUARE_INCHES, 0);
        p5 = new Pose(0, TARGET_SQUARE_INCHES, 0);
        p6 = new Pose(0, 0, 0);


        // Build individual paths with locked headings
        side1 = new Path(new BezierLine(p0, p1));
        side1.setConstantHeadingInterpolation(0);




        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        Pose currentPose = follower.getPose();

        // --- CARTESIAN STATE MACHINE ---
        // This ensures the robot only moves to the next path once the previous is finished.
        switch (pathState) {
            case 0: // Start Side 1
                follower.followPath(side1);
                pathState = 1;
                break;
            case 1: // Waiting to finish Side 1
                if (!follower.isBusy() ) {

                    side2 = new Path(new BezierLine(p1, p2));
                    side2.setConstantHeadingInterpolation(0);

                    follower.followPath(side2);
                    pathState = 2;
                }
                break;
            case 2: // Waiting to finish Side 2
                if (!follower.isBusy()) {
                    //follower.followPath(side3);
                    side3 = new Path(new BezierLine(p1, p2));
                    side3.setConstantHeadingInterpolation(0);

                    follower.followPath(side3);
                    pathState = 3;
                }
                break;
            case 3: // Waiting to finish Side 3
                if (!follower.isBusy() ) {
                   // follower.followPath(side4);
                    side4 = new Path(new BezierLine(p2, p3));
                    side4.setConstantHeadingInterpolation(0);
                    follower.followPath(side4);
                    pathState = 4;
                }
                break;
            case 4: // Waiting to finish Side 4
                if (!follower.isBusy()) {
                    side5 = new Path(new BezierLine(p3, p4));
                    side5.setConstantHeadingInterpolation(0);
                    follower.followPath(side5);
                    pathState = 5; // All Done
                }
                break;
            case 5: // Waiting to finish Side 4
                if (!follower.isBusy()) {
                    side6 = new Path(new BezierLine(p4, p5));
                    side6.setConstantHeadingInterpolation(0);
                    follower.followPath(side6);
                    pathState = 6; // All Done
                }
                break;
            case 6: // Waiting to finish Side 4
                if (!follower.isBusy()) {

                    pathState = 7; // All Done
                }
                break;
        }

        // --- CALC DATA ---
        double fle = flEnc.getVoltage() * VOLTAGE_TO_RAD;
        double fre = frEnc.getVoltage() * VOLTAGE_TO_RAD;
        double ble = blEnc.getVoltage() * VOLTAGE_TO_RAD;
        double bre = brEnc.getVoltage() * VOLTAGE_TO_RAD;

        // --- PANELS TELEMETRY ---
        // pt.addData sends data to your custom Panels interface
       // pt.addData("Path State", pathState);
        //pt.addData("X", currentPose.getX());
        //pt.addData("Y", currentPose.getY());
        //pt.addData("Heading", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Path State", pathState);

        telemetry.addData("X", currentPose.getX());

        telemetry.addData("X", currentPose.getX());

        telemetry.addData("Y", currentPose.getY());

        telemetry.addData("Heading (Deg)", Math.toDegrees(currentPose.getHeading()));

        //telemetry.addData("Target X", p.getX());

        //telemetry.addData("Target Y", targetPose.getY());

        // Standard Telemetry for Dashboard Graphs
        telemetry.addData("FL_Rad", fle);
        telemetry.addData("FR_Rad", fre);
        telemetry.addData("BL_Rad", ble);
        telemetry.addData("BR_Rad", bre);

        // --- FIELD DRAWING ---
        drawToDashboard(currentPose);

        telemetry.update();
    }

    private boolean atTarget(Pose current, double tx, double ty) {
        return Math.hypot(tx - current.getX(), ty - current.getY()) < ARRIVAL_TOLERANCE;
    }

    private void drawToDashboard(Pose currentPose) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStroke("#3F51B5");
        fieldOverlay.strokeRect(currentPose.getX() - (ROBOT_LENGTH / 2),
                currentPose.getY() - (ROBOT_WIDTH / 2),
                ROBOT_LENGTH, ROBOT_WIDTH);

        dashboard.sendTelemetryPacket(packet);
    }
}