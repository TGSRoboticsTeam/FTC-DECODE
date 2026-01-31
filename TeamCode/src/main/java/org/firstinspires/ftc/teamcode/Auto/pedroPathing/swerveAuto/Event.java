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
    PAUSE_LONG,
    LAUNCH_START_THREE, // Triggers the 3-shot routine// Pods at 0 degrees
    PAUSE_1SEC,
    PAUSE_05SEC// 1.0s
}