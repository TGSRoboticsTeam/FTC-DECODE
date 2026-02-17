package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tools.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.Tools.TurretMechanismTutorial;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@Autonomous(name = "BlueFarGoal Feb14", group = "Testing")
public class BlueFarTunedOLD extends OpMode {
    private Follower follower;
    private FtcDashboard dashboard;
    private PanelsTelemetry pt;
    private TurretMechanismTutorial turret;

    // --- Mechanism Hardware ---
    private DcMotor leftFly, rightFly, frontIntake, backIntake;
    private Servo pusher, adjuster;


    // We use individual Paths instead of a PathChain to prevent blending
    private Path side1, side2, side3, side4,side5,side6,side7,side8,side9,side10,side11;
    private Path side31,side32;
    private Path bail;

    private int pathState = 0;

    private AnalogInput flEnc, frEnc, blEnc, brEnc;
    private final double VOLTAGE_TO_RAD = (2 * Math.PI) / 3.3;


    /* ===================== SENSORS (BALL DISTANCE) ===================== */
    private NormalizedColorSensor frontColor, centerColor, backColor; // config compatibility
    private DistanceSensor frontDist, centerDist, backDist;

    boolean frontHasBall=false;
    boolean centerHasBall=false;
    boolean backHasBall=false;

    /* ===================== DISTANCE BALL DETECTION (HYSTERESIS) ===================== */
    final double FRONT_ON_CM  = 4.0, FRONT_OFF_CM  = 5.0;
    final double CENTER_ON_CM = 4.0, CENTER_OFF_CM = 5.0;
    final double BACK_ON_CM   = 4.0, BACK_OFF_CM   = 5.0;


    public static double ROBOT_WIDTH = 18.0;
    public static double ROBOT_LENGTH = 18.0;
    public static double TARGET_SQUARE_INCHES = 12.0;
    public static double ARRIVAL_TOLERANCE = 0.5; // How close before switching paths

    public static double TARGET_TILE_INCHES = 24.0;

    //Define Constants
    public static double TRIGGER_HOME = 0.225;
    public static double TRIGGER_FIRE = 0.0;

   public Timer timer;
   public Timer totalTime;

    private Pose p0,p1,p2,p3,p4,p5,p6,p00,out;

    private Servo lights;
    private AprilTagWebcam aprilTagWebcam;

    private int ballOrder = 21;
    AprilTagDetection target;

    //SETTINGS**********************************//
    //public static final double STEER_KP = 1.0;
    //        public static final double STEER_DEADBAND = 0.10;
    //
     //constants.coefficientsTranslationalPIDF.setCoefficients(TRANS_P, TRANS_I, TRANS_D, 0);
       //     constants.coefficientsHeadingPIDF.setCoefficients(HEAD_P, HEAD_I, HEAD_D, 0);
         //   constants.coefficientsDrivePIDF.setCoefficients(DRIVE_P, DRIVE_I, DRIVE_D, 0.25, 0);
    /// ****************************************

    @Override
    public void init() {
        timer = new Timer();
        totalTime = new Timer();
        initMechanisms();
         turret = new TurretMechanismTutorial();


        aprilTagWebcam.init(hardwareMap, telemetry);

        turret.init(hardwareMap,telemetry,24);
        dashboard = FtcDashboard.getInstance();
        pt = PanelsTelemetry.INSTANCE;
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        follower = SwerveConstants.createFollower(hardwareMap);

        flEnc = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frEnc = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        blEnc = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        brEnc = hardwareMap.get(AnalogInput.class, "backRightEncoder");

        lights = hardwareMap.get(Servo.class, "lights");


        // Define Poses
        p00 = new Pose(0, -7.5, 0); //shooting
        p0 = new Pose(0, -8.5, 0);
        p1 = new Pose(0, -TARGET_TILE_INCHES, 0);//In front of row 1
        p2 = new Pose(30, -TARGET_TILE_INCHES, 0); //Through row 1
        p3 = new Pose(0, -TARGET_TILE_INCHES * 2, 0); //Front row 2
        p4 = new Pose(30, -TARGET_TILE_INCHES * 2, 0);   //Through row 2
        p5 = new Pose(0, -TARGET_TILE_INCHES * 3, 0); //Front row 3
        p6 = new Pose(30, -TARGET_TILE_INCHES * 3, 0);//Through row 3

        out = new Pose(30,-30,0);


        // Build individual paths with locked headings
        side1 = new Path(new BezierLine(p0, p3));
        side1.setConstantHeadingInterpolation(0);



        pathState = -1;
        telemetry.addData("Status", "Swerve Follower Initialized");
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }

    //@Override
    private int count=0;
    public void init_loop() {
       // super.init_loop();
        //count++;
        //telemetry.addLine(" "+count);
        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);//red
        AprilTagDetection id23 = aprilTagWebcam.getTagBySpecificId(23);//purple
        AprilTagDetection id22 = aprilTagWebcam.getTagBySpecificId(22);//orange
        AprilTagDetection id21 = aprilTagWebcam.getTagBySpecificId(21);//green
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);//blue
        target = id20;

        if(id21 != null){
            telemetry.addLine("Tag 21 Detected");
            ballOrder=21;
            lights.setPosition(RGB.green);
        }
        if(id22 != null){
            telemetry.addLine("Tag 22 Detected");
            ballOrder=22;
            lights.setPosition(RGB.orange);
        }
        if(id23 != null){
            telemetry.addLine("Tag 23 Detected");
            ballOrder=23;
            lights.setPosition(RGB.violet);
        }
        telemetry.update();

    }




    private boolean firingComplete = false;
    private boolean hasFired = false;
    @Override
    public void loop() {


        follower.update();

        Pose currentPose = follower.getPose();
        if(totalTime.getElapsedTimeSeconds()>27){
            follower.breakFollowing();
            //Get current Pose
            Pose current = follower.getPose();
            //Create a path from current pose to out
            Path target = new Path(new BezierLine(current, out));
            follower.followPath(target);
            pathState = 99;
        }
        // --- CARTESIAN STATE MACHINE ---
        // This ensures the robot only moves to the next path once the previous is finished.
        switch (pathState) {
            case -2:
                if(hasBall()>0){
                    lights.setPosition(RGB.white);
                };
            break;
            case -1:
                boolean complete = turret.updateUntil(target);
                complete = true;
                if(complete){
                    pathState = 0;
                }
            break;
            case 0: // Start Side 1
                //turret.setServos(.65);
                if(firingComplete){
                    lights.setPosition(RGB.green);
                    follower.followPath(side1);
                    pathState =1;
                }
                if(!hasFired){
                    hasFired = true;
                    turnOnFlys(getfarPower(0.875));//.87
                    adjuster.setPosition(0);

                    fireThreeTimes();
                    firingComplete=true;
                }
                break;
            case 1: // Waiting to finish Side 1
                if (!follower.isBusy() ) {
                    lights.setPosition(RGB.cyan);

                    side2 = new Path(new BezierLine(p3, p4));
                    side2.setConstantHeadingInterpolation(0);


                    follower.followPath(side2);
                    pathState = 2;
                }
                break;
            case 2: // Waiting to finish Side 2
                frontIntake.setPower(.7);
                if (!follower.isBusy()) {
                    //follower.followPath(side3);
                    timer.resetTimer();
                    lights.setPosition(RGB.blue);
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auto/pedroPathing/swerveAuto/BlueFarTunedOLD.java
                    if(timer.getElapsedTimeSeconds() < 2){
                        //backIntake.setPower(-.45);
=======
                    if(timer.getElapsedTimeSeconds() < 2) {
                        backIntake.setPower(-.45);
>>>>>>> 3348a14f4b92d99354a2c17e34c3eed06fd1c22c:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auto/pedroPathing/swerveAuto/BlueFarTuned.java
                    }

                    side3 = new Path(new BezierLine(p4, p3));
                    side3.setConstantHeadingInterpolation(0);




                    follower.followPath(side3);
                    pathState = 3;
                }
                break;
            case 3: // Waiting to finish Side 3

                frontIntake.setPower(0.0);
                backIntake.setPower(0.0);
                if (!follower.isBusy() ) {
                   // follower.followPath(side4);
                    lights.setPosition(RGB.indigo);
                    side31 = new Path(new BezierLine(p3, p1));
                    side31.setConstantHeadingInterpolation(0);
                    side31.setTimeoutConstraint(1000);

                    lights.setPosition(RGB.lime);
                    follower.followPath(side31);
                    pathState = 31;
                }
                break;
            case 31: // Waiting to finish Side 3

               // follower.turnTo(0);
                if (!follower.isBusy() ) {
                    // follower.followPath(side4);
                    lights.setPosition(RGB.white);



                    pathState = 32;
                }
                break;
            case 32: // Waiting to finish Side 3

                frontIntake.setPower(0.0);
                if (!follower.isBusy() ) {
                    // follower.followPath(side4);
                    lights.setPosition(RGB.indigo);
                    side4 = new Path(new BezierLine(p3, p0));
                    side4.setConstantHeadingInterpolation(0);
                    side4.setTimeoutConstraint(300);


                    follower.followPath(side4);
                    pathState = 4;
                }
                break;
            case 4: // Waiting to finish Side 4
               // follower.turnTo(0);
                //if( side4.getDistanceRemaining()<2){
                //    follower.breakFollowing();
                //}
                if (!follower.isBusy()) {
                    turnOnFlys(.875);
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    fireThreeTimes();
                    lights.setPosition(RGB.violet);

                    side5 = new Path(new BezierLine(p00, p1));
                    side5.setConstantHeadingInterpolation(0);

                    follower.followPath(side5);
                    pathState = 5; // All Done
                }
                break;
            case 5: // Waiting to finish Side 4
                if (!follower.isBusy()) {
                    lights.setPosition(RGB.red);
                    side6 = new Path(new BezierLine(p1, p2));
                    side6.setConstantHeadingInterpolation(0);
                    follower.followPath(side6);
                    pathState = 6; // All Done
                }
                break;
            case 6: // Waiting to finish Side 4
                if (!follower.isBusy()) {
                    lights.setPosition(RGB.red);
                    side7 = new Path(new BezierLine(p2, p1));
                    side7.setConstantHeadingInterpolation(0);

                    follower.followPath(side7);
                    pathState = 7; // All Done
                }
                break;
            case 7: // Waiting to finish Side 4
                if (!follower.isBusy()) {
                    lights.setPosition(RGB.red);
                    side8 = new Path(new BezierLine(p1, p00));
                    side8.setConstantHeadingInterpolation(0);

                    follower.followPath(side8);
                    pathState = 8; // All Done
                }
                break;
            case 8: // Waiting to finish Side 4
                if (!follower.isBusy()) {
                    lights.setPosition(RGB.red);
                    side9= new Path(new BezierLine(p00, p5));
                    side9.setConstantHeadingInterpolation(0);

                    follower.followPath(side9);
                    pathState = 9; // All Done
                }
                break;
            case 9: // Waiting to finish Side 4
                if (!follower.isBusy()) {
                    lights.setPosition(RGB.red);
                    side10 = new Path(new BezierLine(p5, p6));
                    side10.setConstantHeadingInterpolation(0);

                    follower.followPath(side10);
                    pathState = 10; // All Done
                }
                break;
            case 10: // Waiting to finish Side 4
                if (!follower.isBusy()) {
                    lights.setPosition(RGB.red);
                    side11 = new Path(new BezierLine(p6, p5));
                    side11.setConstantHeadingInterpolation(0);

                    follower.followPath(side6);
                    pathState = 11; // All Done
                }
                break;
            case 11: // Waiting to finish Side 4
                if (!follower.isBusy()) {
                    lights.setPosition(RGB.red);
                    side8 = new Path(new BezierLine(p5, p0));
                    side8.setConstantHeadingInterpolation(0);

                    follower.followPath(side6);
                    pathState = 10; // All Done
                }
                break;
            case 12: // Waiting to finish Side 4
                if (!follower.isBusy()) {
                    lights.setPosition(RGB.orange);
                    pathState = 11; // All Done
                }
                break;
            case 99: // Waiting to finish Side 4
                if (!follower.isBusy()) {
                    lights.setPosition(RGB.blue);
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
    public double getfarPower(double p){
        //Get the voltage of battery\
        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        return p * 12.0/voltage;
    }

    private int hasBall(){  /* ===================== BALL DISTANCE SENSING ===================== */
        double fCm = safeDistanceCm(frontDist);
        double cCm = safeDistanceCm(centerDist);
        double bCm = safeDistanceCm(backDist);

        frontHasBall  = hysteresisBall(frontHasBall,  fCm, FRONT_ON_CM,  FRONT_OFF_CM);
        centerHasBall = hysteresisBall(centerHasBall, cCm, CENTER_ON_CM, CENTER_OFF_CM);
        backHasBall   = hysteresisBall(backHasBall,   bCm, BACK_ON_CM,   BACK_OFF_CM);

        if(frontHasBall && centerHasBall && backHasBall){
            return 7;
        } else if (frontHasBall && centerHasBall) {
            return 3;
        } else if (frontHasBall && backHasBall) {
            return 4;
        } else if (centerHasBall && backHasBall) {
            return 5;
        } else if (frontHasBall) {
            return 1;
        } else if (centerHasBall) {
            return 2;
        } else if (backHasBall ) {
            return 3;
        }
        return 0;

    }

    /* ===================== DISTANCE HELPERS ===================== */
    private double safeDistanceCm(DistanceSensor s) {
        if (s == null) return 999.0;
        double cm = s.getDistance(DistanceUnit.CM);
        if (Double.isNaN(cm) || Double.isInfinite(cm)) return 999.0;
        return cm;
    }


    private boolean hysteresisBall(boolean prev, double cm, double onCm, double offCm) {
        return prev ? (cm <= offCm) : (cm <= onCm);
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

    private void initMechanisms() {
        leftFly = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");
        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");
        pusher = hardwareMap.get(Servo.class, "trigger");
        adjuster = hardwareMap.get(Servo.class, "adjuster");
        aprilTagWebcam = new AprilTagWebcam();
    }
    private void turnOnFlys(double power){
        telemetry.addData("Power", power);
        telemetry.update();
        leftFly.setPower(-power);
        rightFly.setPower(power);
    }

    private void fireThreeTimes() {
        try {
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auto/pedroPathing/swerveAuto/BlueFarTunedOLD.java
            Thread.sleep(400);
=======
            Thread.sleep(700);
>>>>>>> 3348a14f4b92d99354a2c17e34c3eed06fd1c22c:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auto/pedroPathing/swerveAuto/BlueFarTuned.java
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        fireOne();
        if(ballOrder==21 ||ballOrder==22){
            frontIntake.setPower(0.65);

        }else{
        backIntake.setPower(-0.65);
            }
        try {
<<<<<<< HEAD:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auto/pedroPathing/swerveAuto/BlueFarTunedOLD.java
            Thread.sleep(200);
=======
            Thread.sleep(900);
>>>>>>> 3348a14f4b92d99354a2c17e34c3eed06fd1c22c:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auto/pedroPathing/swerveAuto/BlueFarTuned.java
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        frontIntake.setPower(0);
        backIntake.setPower(0);
        fireOne();
        if(ballOrder==21 ||ballOrder==22){
            backIntake.setPower(-0.65);

        }else{
            frontIntake.setPower(0.65);
        }
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        backIntake.setPower(0);
        frontIntake.setPower(0);
        fireOne();
        leftFly.setPower(0.0);
        rightFly.setPower(0.0);
    }
    public void fireOne()  {
        pusher.setPosition(TRIGGER_FIRE);
        try {
            Thread.sleep(400);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        pusher.setPosition(TRIGGER_HOME);
        try {
            Thread.sleep(400);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}