package org.firstinspires.ftc.teamcode.pedroPathing.conciseAuto;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.pedroPathing.conciseAuto.FieldPose.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.conciseAuto.FieldPaths.*;


//import Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;




@Autonomous(name = "Eventful Auto", group = "Examples")
public class EventfulAuto extends OpMode {

    private Follower follower;
    private int pathIndex = 0;

    // The new sequence array, holding both paths and events
    private PathAndEvent[] pathSequence;

    @Override
    public void init() {
        // Initialize the follower and field paths
        follower = Constants.createFollower(hardwareMap);
        FieldPaths.initializePaths(follower);
        follower.setStartingPose(redDepot);

        // Initialize the FieldEvent class with the hardware map
        FieldEvent.initialize(hardwareMap);

        // Define the path sequence with an event after each path
/*
        pathSequence = new PathAndEvent[]{
                new PathAndEvent(redDepotShoot, Event.SHOOT),
                new PathAndEvent(redShootCenter, Event.CALIBRATE),
                new PathAndEvent(CENTER_F2, Event.NULL),
                new PathAndEvent(F2_CENTER, Event.NULL), // Pause for 2 seconds
                new PathAndEvent(F2_CENTER, Event.NULL), // No event
                new PathAndEvent(CENTER_A5, Event.CALIBRATE),
                new PathAndEvent(A5_CENTER, Event.NULL),
                new PathAndEvent(CENTER_F5, Event.READ),
                new PathAndEvent(F5_CENTER, Event.NULL) // No event
        };
*/

        pathSequence = new PathAndEvent[]{
                new PathAndEvent(Colorado, Event.NULL),
                new PathAndEvent(NewMexico, Event.NULL)

        };



    }
    @Override
    public void start() {
        pathIndex = 0;
        // Start the first path immediately
        if (pathIndex < pathSequence.length) {
            follower.followPath(pathSequence[pathIndex].path);
        }
    }

    @Override
    public void loop() {
        follower.update();

        // If the current path is complete, perform the event and move to the next path
        if (!follower.isBusy()) {
            if (pathIndex < pathSequence.length) {
                // Perform the event associated with the completed path
                FieldEvent.perform(pathSequence[pathIndex].event);

                // Then, move to the next path in the sequence
                pathIndex++;

                // Follow the next path if it exists
                if (pathIndex < pathSequence.length) {
                    follower.followPath(pathSequence[pathIndex].path);
                } else {
                    // All paths are complete, set index to a final state
                    pathIndex = -1;
                }
            }
        }

        // Telemetry for debugging
        telemetry.addData("Path Index", pathIndex);
        telemetry.addData("Current Event", pathIndex > -1 && pathIndex < pathSequence.length ? pathSequence[pathIndex].event : "N/A");
        telemetry.update();
    }

    @Override
    public void stop() {
        // You can add logic here to stop all subsystems
    }
}