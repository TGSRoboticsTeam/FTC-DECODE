package org.firstinspires.ftc.teamcode.Tools;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto.RGB;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


public class TurretMechanismTutorial {
    // Hardware
    private Servo turretServo1;
    private Servo turretServo2;
    private Servo lights;

    // Constants & State
    // kP is now "Sensitivity". Start VERY small for servos (e.g., 0.001)
    // It represents how much servo position changes per degree of error.
    private double kP = 0.0003;

    // Track the servo's current commanded position (0.0 to 1.0)
    private double currentServoPos = 0.5;

    // Tolerances
    private double angleTolerance = 3; // Degrees of deadzone
    Telemetry telemetry;

    public void init(HardwareMap hwMap,Telemetry t) {
        telemetry = t;
        turretServo1 = hwMap.get(Servo.class, "turret_rotation_1");
        turretServo2 = hwMap.get(Servo.class, "turret_rotation_2");
        lights = hwMap.get(Servo.class, "lights");


        // Assuming servos are mounted opposite each other
        turretServo1.setDirection(Servo.Direction.FORWARD);
        turretServo2.setDirection(Servo.Direction.REVERSE);

        // Initialize to center
        currentServoPos = 0.5;
        setServos(currentServoPos);
    }
    double scroll = 0;
    double scrollIncrement = 0.001;
    public void update(AprilTagDetection curID) {
        // Safety: If no tag is seen, do nothing (or return to center if preferred)
        if (curID == null) {
            if(scroll <=0.1 && scrollIncrement<0){
                scrollIncrement = -1*scrollIncrement;
            }
            if(scroll >=0.9 && scrollIncrement>0){
                scrollIncrement = -1*scrollIncrement;
            }
            scroll += scrollIncrement;
            telemetry.addLine("No Tag Detected. Stopping Turret Motor: "+scroll);
            setServos(scroll);
            return;
        }
        else {
            // 1. Calculate Error
            // AprilTags give 'bearing' in degrees.
            // If bearing is positive (left), we might need to increase position.
            // You may need to flip the sign (-) depending on your servo orientation.
            // double error = -curID.ftcPose.bearing;
            double error = curID.ftcPose.elevation;  //sign depends on rotation  , using elevation because rotated

            // 2. Deadzone check
            // If we are close enough, stop updating to prevent buzzing
            if (Math.abs(error) < angleTolerance) {
                lights.setPosition(RGB.green);
                return;
            }
            lights.setPosition(RGB.yellow);
            // 3. Calculate "Step" (Proportional control on the RATE of change)
            // Large error = large step; Small error = small step
            double step = error/12;
           // double step = 0.0001;
            // 4. Update Position
            scroll = currentServoPos;
            currentServoPos += step;

            // 5. Safety Clamp
            // Keep the signal within the valid 0.0 to 1.0 range
            currentServoPos = Range.clip(currentServoPos, 0.0, 1.0);

            telemetry.addLine("Current Servo Pos: " + currentServoPos);
            telemetry.addLine("Error: " + error);
            telemetry.update();


            // 6. Apply to Hardware
            setServos(currentServoPos);
            //print current servoPos  and error
        }

    }

    // Helper method to keep things clean
    private void setServos(double pos) {
        turretServo1.setPosition(pos);
        turretServo2.setPosition(1-pos); // Since direction is reversed, this matches
    }

    // Getters and Setters for tuning
    public void setkP(double newKP) {
        this.kP = newKP;
    }
}