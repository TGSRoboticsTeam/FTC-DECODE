package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;

import static java.lang.Thread.sleep;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.RGB;
import org.firstinspires.ftc.teamcode.Tools.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.Tools.TurretMechanism;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.pedropathing.util.Timer;

@Autonomous(name = "RandoRed", group = "Testing")
public class RandoRed extends OpMode {
    private Follower follower;



    // --- Mechanism Hardware ---
    private DcMotor leftFly, rightFly, frontIntake, backIntake;
    private Servo pusher, adjuster;

    private TurretMechanism turret;

    private Servo lights;

    private AprilTagWebcam aprilTagWebcam;
    private AprilTagDetection targetID;

    //Swerve Servo Encoders
    private AnalogInput flEnc, frEnc, blEnc, brEnc;

    // --- Ball sensors (config names: frontColor/centerColor/backColor) ---
    private DistanceSensor frontDist, centerDist, backDist;

    // We use individual Paths instead of a PathChain to prevent blending
    private Path side1, side2, side3, side4,side5,side6,side7,side8,side9,side10,side11,side31,side32;
    private Pose p0,p1,p2,p3,p4,p5,p6,p00, pc;

    private int pathState = 0;

    //Default AprilTag ID
    private int ballOrder = 21;


    private final double VOLTAGE_TO_RAD = (2 * Math.PI) / 3.3;

    public static double ROBOT_WIDTH = 18.0;
    public static double ROBOT_LENGTH = 18.0;
    public static double TARGET_SQUARE_INCHES = 12.0;
    public static double ARRIVAL_TOLERANCE = 0.5; // How close before switching paths

    public static double TARGET_TILE_INCHES = 24.0;

    //Define Constants
    public static double TRIGGER_HOME = 0.225;
    public static double TRIGGER_FIRE = 0.0;

    public Timer timer;
    public Timer gameTimer;

    // Add this as a global variable in your Auto class
    private Timer settleTimer = new Timer();
    private boolean centering = false;




    // Outward burp (TeleOp)
    public static double SPINUP_OUTWARD_POWER = 0.4;
    public static long SPINUP_OUTWARD_MS = 40;

    // Launch feeding (TeleOp)
    public static double FRONT_FEED_POWER = 0.65;

    // TeleOp uses BACK_FEED_POWER = 0.375, then applies negative sign when feeding
    public static double BACK_FEED_POWER = 0.375;

    // Third shot feed slower (your request)
    public static double BACK_FEED_POWER_THIRD = BACK_FEED_POWER * 1; // tune (0.80..0.95)

    public static long FEED_TO_CENTER_MS = 800;//change from 450

    // Trigger timing (TeleOp)
    public static long LAUNCH_TRIGGER_HOLD_MS = 300;
    public static long TRIGGER_RESET_WAIT_MS = 125;
    public static long RETRY_EXTRA_WAIT_MS = 350;

    // Spin timing (TeleOp)
    public static long FLYWHEEL_SPINUP_MS = 1000;
    public static long FLYWHEEL_SPINDOWN_MS = 300;

    // Retry control (TeleOp)
    public static int MAX_RETRIES_PER_SHOT = 3;

    private boolean hasFired = false;
    private boolean firingComplete = false;

    // ===================== CENTER-ONLY BALL DETECTION (HYSTERESIS) =====================
    // Only CENTER sensor is used to decide whether a retry is needed.
    public static double CENTER_ON_CM = 4.0, CENTER_OFF_CM = 5.0;
    public static double FRONT_ON_CM = 4.0, FRONT_OFF_CM = 5.0;
    public static double BACK_ON_CM = 4.0, BACK_OFF_CM = 5.0;
    private boolean centerHasBall = false;
    private boolean frontHasBall = false;
    private boolean backHasBall = false;

    // ==================================  ==============================================


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
        gameTimer = new Timer();

        initMechanisms();
        turret = new TurretMechanism();
        aprilTagWebcam.init(hardwareMap, telemetry);
        turret.init(hardwareMap,telemetry,24); //Blue is 20, red is 24  *Change for Red



        follower = SwerveConstants.createFollower(hardwareMap);


        // Define Poses
        p00 = new Pose(-5, -3.5, 0); //shooting
        p0 = new Pose(0, -8.5, 0);
        p1 = new Pose(0, -TARGET_TILE_INCHES, 0);//In front of row 1
        p2 = new Pose(-30, -TARGET_TILE_INCHES, 0); //Through row 1
        p3 = new Pose(0, -TARGET_TILE_INCHES * 2, 0); //Front row 2
        p4 = new Pose(-30, -TARGET_TILE_INCHES * 2, 0);   //Through row 2
        p5 = new Pose(0, -TARGET_TILE_INCHES * 3, 0); //Front row 3
        p6 = new Pose(-30, -TARGET_TILE_INCHES * 3, 0);//Through row 3
        pc = new Pose(-30, -8.5, 0); //clean corner

        follower.setStartingPose(p0);

        // Build individual paths with locked headings
        side1 = new Path(new BezierLine(p0, p1));
        side1.setConstantHeadingInterpolation(0);

        pathState = -3;

        telemetry.addData("Status", "Swerve Follower Initialized");
        telemetry.addData("Path State", pathState);
        telemetry.update();
    }

    //@Override
    private int count=0;
    public void init_loop() {

        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);//red
        AprilTagDetection id23 = aprilTagWebcam.getTagBySpecificId(23);//purple
        AprilTagDetection id22 = aprilTagWebcam.getTagBySpecificId(22);//orange
        AprilTagDetection id21 = aprilTagWebcam.getTagBySpecificId(21);//green
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);//blue
        targetID = id24;

        gameTimer.resetTimer();

        if(id21 != null){
            telemetry.addLine("Tag 21 Detected");
            ballOrder=21;
            lights.setPosition(RGB.green);
        }
        else  if(id22 != null){
            telemetry.addLine("Tag 22 Detected");
            ballOrder=22;
            lights.setPosition(RGB.orange);
        }
        else if(id23 != null){
            telemetry.addLine("Tag 23 Detected");
            ballOrder=23;
            lights.setPosition(RGB.violet);
        }
        else if(id20!=null){
            telemetry.addLine("Only Blue Detected");
            ballOrder = 21;
            lights.setPosition(RGB.blue);

        }
        else if(id24!=null){
            telemetry.addLine("Only Red Detected");
            ballOrder = 21;
            lights.setPosition(RGB.red);

        }
        telemetry.update();

    }

    private int mcount=0;
    @Override
    public void loop() {


        if(gameTimer.getElapsedTimeSeconds()>28){
            follower.breakFollowing();
            pathState = 99;
        }

        follower.update();
        Pose currentPose = follower.getPose();


        if(currentPose.getHeading()>.5){
            lights.setPosition(RGB.white);
            // follower.breakFollowing();
            //follower.setTeleOpDrive(0,0,Math.toRadians(10));
        }

        switch (pathState) {
            case -7:
                lights.setPosition(RGB.orange);
                // 1. Stop any active PedroPathing movement
                follower.breakFollowing();

                // 2. Access your custom drivetrain method
                SwerveDrivetrain drive = (SwerveDrivetrain) follower.getDrivetrain();


                // 3. Call the correction logic (Target 0 degrees)
                boolean isAligned = drive.forceHeadingCorrection(0.0, follower.getPose().getHeading());
                lights.setPosition(RGB.red);
                // ONLY break following if we just got here
                if (follower.isBusy()) {
                    follower.breakFollowing();
                }
                ;                // 4. Transition once the robot is straight
                if (isAligned) {

                    lights.setPosition(RGB.green);
                    pathState = 0;
                    // Optional: Reset the follower's internal heading to exactly 0 to clear drift
                    // follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
                }

                break;


// Inside your switch block
            case -3:

                SwerveDrivetrain adrive = (SwerveDrivetrain) follower.getDrivetrain();
                boolean isAlignedb = adrive.forceHeadingCorrection(0.0, follower.getPose().getHeading());
                telemetry.addLine("READING: "+follower.getPose().getHeading()+"     "+mcount);
                mcount++;
                if (isAlignedb) {
                    if (!centering) {
                        settleTimer.resetTimer();
                        centering = true;
                    }
                    lights.setPosition(RGB.green);

                    // Only transition if we stay green for 150ms
                    if (settleTimer.getElapsedTime() > 150) {
                        adrive.breakFollowing();
                        pathState = 0;
                    }
                } else {
                    centering = false;
                    lights.setPosition(RGB.red);
                }
                break;
            case -2:

                if (!follower.isBusy() ) {
                    lights.setPosition(RGB.white);
                }




                break;
            case -1:
                // boolean complete = turret.updateUntil(targetID);
                if (!follower.isBusy() ) {
                    pathState=0;
                }
                //complete = true;
                //  if(complete){
                //      pathState = 0;
                //  }
                break;
            case 0: // First Fire
                turret.setServos(.45);
                if(firingComplete){
                    lights.setPosition(RGB.green);
                    follower.followPath(side1);
                    pathState =1;
                }
                if(!hasFired){
                    hasFired = true;
                    turnOnFlys(getfarPower(0.75));//.87
                    adjuster.setPosition(0);

                    fireThree(getfarPower(0.75));
                    firingComplete=true;

                }
                if(hasBall()>0){
                    frontIntake.setPower(-.5);
                    try {
                        Thread.sleep(30);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    frontIntake.setPower(0);
                }
                break;
            case 1: // Waiting to finish first path, then start second path
                backIntake.setPower(.65);
                if (!follower.isBusy() ) {
                    lights.setPosition(RGB.cyan);

                    side2 = new Path(new BezierLine(p1, p2));
                    side2.setConstantHeadingInterpolation(0);


                    follower.followPath(side2);
                    pathState = 2;
                }
                break;
            case 2: // Waiting to finish path 2 starting path 3(pickup row)
                backIntake.setPower(.65);
                if (!follower.isBusy()) {
                    //follower.followPath(side3);
                    lights.setPosition(RGB.blue);

                    side3 = new Path(new BezierLine(p2, p1));
                    side3.setConstantHeadingInterpolation(0);


                    follower.followPath(side3);
                    pathState = 3;
                }
                break;
            case 3: //finishing path 2(Could use to hit gate

                frontIntake.setPower(0.0);
                backIntake.setPower(0.0);

                if (!follower.isBusy() ) {
                    // follower.followPath(side4);
                    lights.setPosition(RGB.indigo);

                    //side31 = new Path(new BezierLine(p3, p1));
                    //side31.setConstantHeadingInterpolation(0);


                    // follower.followPath(side31);
                    pathState = 31;
                }
                break;
            case 31: // correction step
                // 1. Stop any active PedroPathing movement
                follower.breakFollowing();

                // 2. Access your custom drivetrain method
                SwerveDrivetrain rDrive = (SwerveDrivetrain) follower.getDrivetrain();

                // 3. Call the correction logic (Target 0 degrees)
                boolean isAlignedAngle = rDrive.forceHeadingCorrection(0.0, follower.getPose().getHeading());

                // 4. Transition once the robot is straight
                if (isAlignedAngle) {
                    pathState = 32;
                    // Optional: Reset the follower's internal heading to exactly 0 to clear drift
                    follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
                }
                break;

            case 32: // Waiting to finish Side 3

                backIntake.setPower(0.0);

                if (!follower.isBusy() ) {
                    // follower.followPath(side4);
                    lights.setPosition(RGB.indigo);
                    side4 = new Path(new BezierLine(p1, p00));
                    side4.setConstantHeadingInterpolation(0);


                    follower.followPath(side4);
                    pathState = 4;
                }
                break;
            case 4: // Fire again - Second Fire

                if (!follower.isBusy()) {
                    timer.resetTimer();
                    turret.setServos(-0.65);

                    aprilTagWebcam.update();
                    targetID = aprilTagWebcam.getTagBySpecificId(24);//blue

                    while(!turret.updateUntil(targetID)){

                        aprilTagWebcam.update();
                        targetID = aprilTagWebcam.getTagBySpecificId(24);//blue

                        if(timer.getElapsedTimeSeconds()>3){
                            break;
                        }
                    }
                    fireThree(getfarPower(0.75));
                    if(hasBall()>0){
                        fireThree(getfarPower(0.75));
                    }
                    timer.resetTimer();
                    lights.setPosition(RGB.orange);

                    // follower.followPath(side5);
                    pathState = 42; // All Done
                }
                break;
            case 42:
                if(timer.getElapsedTimeSeconds()>2){
                    follower.breakFollowing();
                }
                follower.setPose(p00);
                if (!follower.isBusy()) {
                    // side5 = new Path(new BezierLine(p1, p00));
                    //  side5.setConstantHeadingInterpolation(0);

                    pathState = 5;
                }
                break;
            case 5: // Waiting to finish Side 1
                if (!follower.isBusy() ) {
                    lights.setPosition(RGB.cyan);

                    //Pose phalf = new Pose(currentPose.getX()+12, currentPose.getY(),0);
                    side6 = new Path(new BezierLine(p00, pc));
                    side6.setConstantHeadingInterpolation(0);

                    backIntake.setPower(.65);
                    follower.followPath(side6);
                    timer.resetTimer();
                    pathState = 6;
                }
                break;

            case 6: // Waiting to finish Side 2
                //if(timer.getElapsedTimeSeconds()>2){
                //      follower.breakFollowing();
                //  }


                if (!follower.isBusy()) {
                    //follower.followPath(side3);
                    lights.setPosition(RGB.blue);
                    backIntake.setPower(0.0);

                    side7 = new Path(new BezierLine(pc, p00));
                    side7.setConstantHeadingInterpolation(0);


                    follower.followPath(side7);
                    pathState = 7;
                }
                break;
            case 7: // Start Side 1
                if (!follower.isBusy()) {
                    if (firingComplete) {
                        lights.setPosition(RGB.green);
                        follower.followPath(side6);
                        pathState = 8;
                    }
                    timer.resetTimer();
                    //turret.resetScroll();;
                    while (!turret.updateUntil(targetID)) {
                        aprilTagWebcam.update();
                        if ((timer.getElapsedTimeSeconds() > 3)||(gameTimer.getElapsedTimeSeconds()>28)) {
                            break;
                        }
                    }

                    if (!hasFired) {
                        hasFired = true;
                        turnOnFlys(getfarPower(0.8));//.87
                        adjuster.setPosition(0);

                        fireThree(getfarPower(0.8));
                        firingComplete = true;
                    }
                }
                break;
            case 8: // Waiting to finish Side 1
                if (!follower.isBusy() ) {
                    lights.setPosition(RGB.cyan);

                    side6 = new Path(new BezierLine(p00, pc));
                    side6.setConstantHeadingInterpolation(0);


                    follower.followPath(side6);
                    pathState = 9;
                }
                break;
            case 9: // Waiting to finish Side 2
                backIntake.setPower(.65);
                if (!follower.isBusy()) {
                    //follower.followPath(side3);
                    lights.setPosition(RGB.blue);

                    side7 = new Path(new BezierLine(pc, p00));
                    side7.setConstantHeadingInterpolation(0);


                    follower.followPath(side7);
                    pathState = 10;
                }
                break;
            case 10: // Start Side 1
                //turret.setServos(.65);
                if(firingComplete){
                    lights.setPosition(RGB.green);
                    follower.followPath(side6);
                    pathState =1;
                }
                if(!hasFired){
                    hasFired = true;
                    turnOnFlys(getfarPower(0.8));//.87
                    adjuster.setPosition(0);

                    fireThree(getfarPower(0.8));
                    firingComplete=true;
                }
                follower.breakFollowing();
                pathState = 99;
                break;

            case 99: // Waiting to finish Side 4
                lights.setPosition(RGB.white);
                stopIntakes();

                if (!follower.isBusy()) {
                    lights.setPosition(RGB.blue);
                    Pose p = follower.getPose();
                    Path pat = new Path(new BezierLine(p, p2));
                    follower.followPath(pat);
                    pathState=100;



                }
                break;
        }


        // --- CALC DATA ---
        double fle = flEnc.getVoltage() * VOLTAGE_TO_RAD;
        double fre = frEnc.getVoltage() * VOLTAGE_TO_RAD;
        double ble = blEnc.getVoltage() * VOLTAGE_TO_RAD;
        double bre = brEnc.getVoltage() * VOLTAGE_TO_RAD;


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



    private void initMechanisms() {
        leftFly = hardwareMap.get(DcMotor.class, "leftFly");
        rightFly = hardwareMap.get(DcMotor.class, "rightFly");

        frontIntake = hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = hardwareMap.get(DcMotor.class, "backIntake");

        pusher = hardwareMap.get(Servo.class, "trigger");
        adjuster = hardwareMap.get(Servo.class, "adjuster");

        aprilTagWebcam = new AprilTagWebcam();

        // Your sensor names:
        frontDist  = hardwareMap.get(DistanceSensor.class, "frontColor");
        centerDist = hardwareMap.get(DistanceSensor.class, "centerColor");
        backDist   = hardwareMap.get(DistanceSensor.class, "backColor");

        flEnc = hardwareMap.get(AnalogInput.class, "frontLeftEncoder");
        frEnc = hardwareMap.get(AnalogInput.class, "frontRightEncoder");
        blEnc = hardwareMap.get(AnalogInput.class, "backLeftEncoder");
        brEnc = hardwareMap.get(AnalogInput.class, "backRightEncoder");

        lights = hardwareMap.get(Servo.class, "lights");
    }
    private void turnOnFlys(double power){
        telemetry.addData("Power", power);
        telemetry.update();
        leftFly.setPower(-power);
        rightFly.setPower(power);
    }

    // =========================
    // fireThree: ALWAYS 3 shots
    // Retry decision: CENTER sensor only
    // =========================
    private void fireThree(double flyCmd) {
        telemetry.addLine("=== fireThree: ALWAYS 3 shots (center retry only) ===");
        telemetry.update();

        // Start with BURP then SPINUP (TeleOp order)
        if (pusher != null) pusher.setPosition(TRIGGER_HOME);
        setFlyHold(flyCmd);
        stopIntakes();

        // 1) BURP outward
        outwardBurp();

        // 2) SPINUP wait
        sleepMs(FLYWHEEL_SPINUP_MS);

        if(gameTimer.getElapsedTimeSeconds()>28){
            follower.breakFollowing();
            pathState = 99;
            stopIntakes();
            setFlyHold(0.0);
            return;
        }

        // 3) SHOT 1: FIRE center (retry uses center sensor only)
        fireOneWithCenterOnlyRetry(flyCmd);

        // 4) SHOT 2: FEED FRONT -> FIRE (retry uses center sensor only)
        if(ballOrder==21 ||ballOrder==22){
            feedFrontToCenter();
        }else{
            feedBackToCenterSlower();
        }

        fireOneWithCenterOnlyRetry(flyCmd);
        fireOneWithCenterOnlyRetry(flyCmd);

        // 5) SHOT 3: FEED BACK (slower) -> FIRE (retry uses center sensor only)
        if(ballOrder==21 ||ballOrder==22){
            feedBackToCenterSlower();
        }else{
            feedFrontToCenter();

        }
        if(gameTimer.getElapsedTimeSeconds()>28){
            follower.breakFollowing();
            pathState = 99;
            stopIntakes();
            setFlyHold(0.0);

            return;
        }
        fireOneWithCenterOnlyRetry(flyCmd);
        if(hasBall()>0){
            fireOneWithCenterOnlyRetry(flyCmd);
        }
        // 6) SPINDOWN
        stopIntakes();
        if (pusher != null) pusher.setPosition(TRIGGER_HOME);
        sleepMs(FLYWHEEL_SPINDOWN_MS);
        setFlyHold(0.0);
    }

    /**
     * Matches TeleOp LaunchState.BURP behavior:
     * frontIntake = -SPINUP_OUTWARD_POWER
     * backIntake  = +SPINUP_OUTWARD_POWER
     * wait SPINUP_OUTWARD_MS then stop.
     */
    private void outwardBurp() {
        if (pusher != null) pusher.setPosition(TRIGGER_HOME);

        if (frontIntake != null) frontIntake.setPower(-SPINUP_OUTWARD_POWER);
        if (backIntake != null)  backIntake.setPower(+SPINUP_OUTWARD_POWER);

        sleepMs(SPINUP_OUTWARD_MS);
        stopIntakes();
    }
    private void setFlyHold(double power) {
        // Keep your auto convention: leftFly negative, rightFly positive
        if (leftFly != null) leftFly.setPower(-power);
        if (rightFly != null) rightFly.setPower(+power);
    }

    /**
     * TeleOp-style retry loop, BUT ONLY checks CENTER sensor (hysteresis centerHasBall).
     */
    private void fireOneWithCenterOnlyRetry(double flyCmd) {
        int retries = 0;

        while (true) {
            setFlyHold(flyCmd);
            stopIntakes();

            // FIRE hold
            if (pusher != null) pusher.setPosition(TRIGGER_FIRE);
            sleepMs(LAUNCH_TRIGGER_HOLD_MS);

            // RESET wait
            if (pusher != null) pusher.setPosition(TRIGGER_HOME);
            sleepMs(TRIGGER_RESET_WAIT_MS);

            // Check center only
            updateCenterPresence();

            if (!centerHasBall) {
                // success
                return;
            }

            // still has ball -> extra wait then retry (TeleOp behavior)
            sleepMs(RETRY_EXTRA_WAIT_MS);

            retries++;
            if (retries >= MAX_RETRIES_PER_SHOT) {
                // abort this shot
                stopIntakes();
                setFlyHold(0.0);
                if (pusher != null) pusher.setPosition(TRIGGER_HOME);
                return;
            }
        }
    }
    // =========================
    // CENTER-ONLY presence (hysteresis)
    // =========================
    private void updateCenterPresence() {
        double cCm = safeDistanceCm(centerDist);
        centerHasBall = hysteresisBall(centerHasBall, cCm, CENTER_ON_CM, CENTER_OFF_CM);
    }

    private double safeDistanceCm(DistanceSensor s) {
        if (s == null) return 999.0;
        double cm = s.getDistance(DistanceUnit.CM);
        if (Double.isNaN(cm) || Double.isInfinite(cm)) return 999.0;
        return cm;
    }

    private boolean hysteresisBall(boolean prev, double cm, double onCm, double offCm) {
        return prev ? (cm <= offCm) : (cm <= onCm);
    }

    private void feedFrontToCenter() {
        if (pusher != null) pusher.setPosition(TRIGGER_HOME);

        if (frontIntake != null) frontIntake.setPower(+FRONT_FEED_POWER);
        if (backIntake != null)  backIntake.setPower(0);

        sleepMs(FEED_TO_CENTER_MS);
        stopIntakes();
    }

    private void feedBackToCenterSlower() {
        if (pusher != null) pusher.setPosition(TRIGGER_HOME);

        // back feed direction is negative (TeleOp style)
        if (backIntake != null)  backIntake.setPower(-BACK_FEED_POWER_THIRD);
        if (frontIntake != null) frontIntake.setPower(0);

        sleepMs(FEED_TO_CENTER_MS);
        stopIntakes();
    }

    private void stopIntakes() {
        if (frontIntake != null) frontIntake.setPower(0);
        if (backIntake != null) backIntake.setPower(0);
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

    private void sleepMs(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

}