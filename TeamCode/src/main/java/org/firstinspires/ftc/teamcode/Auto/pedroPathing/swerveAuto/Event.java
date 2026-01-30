package org.firstinspires.ftc.teamcode.Auto.pedroPathing.swerveAuto;


public enum Event {
    NULL,
    SHOOT,
    LAUNCH_THREE, // New: Handles the 3-shot cycle
    INTAKE_ON,
    INTAKE_OFF,
    ALIGN_LEFT_RIGHT, // Swerve pod positioning
    ALIGN_FORWARD_BACK,
    ALIGN_DIAGONAL,
    PAUSE_SHORT, // 0.5s
    PAUSE_LONG    // 1.0s
}