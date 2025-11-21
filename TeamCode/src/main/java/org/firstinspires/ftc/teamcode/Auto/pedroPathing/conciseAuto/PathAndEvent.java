package org.firstinspires.ftc.teamcode.Auto.pedroPathing.conciseAuto;

import com.pedropathing.paths.PathChain;

// --- Helper class to store a path, an event, and a duration ---
class PathAndEvent {
    public final PathChain path;
    public final Event event;
    public final double duration;

    public PathAndEvent(PathChain path, Event event) {
        this(path, event, 0.0);
    }

    public PathAndEvent(PathChain path, Event event, double duration) {
        this.path = path;
        this.event = event;
        this.duration = duration;
    }
}