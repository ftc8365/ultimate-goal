package org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling;

public class Motion {
    public enum Type {
        MOVE_FORWARD,
        MOVE_BACKWARD,
        STRAFE_RIGHT,
        STRAFE_LEFT,
        TURN_RIGHT,
        TURN_LEFT,
        STOP
    };

    private final Type      type;
    private final double    distance;
    private final int       targetHeading;

    private Constraint      constraint;
    private boolean         completed;

    Motion(Type type, double distance, int targetHeading, Constraint constraint ) {
        this.type           = type;
        this.distance       = distance;
        this.targetHeading  = targetHeading;
        this.constraint     = constraint;
        this.completed      = false;
    }

    public void setCompleted() {
        this.completed = true;
    }

    public boolean isCompleted() {
        return this.completed;
    }

    public Type getType() {
        return this.type;
    }

    public double getDistance() {
        return this.distance;
    }

    public int getTargetHeading() {
        return this.targetHeading;
    }

    public Constraint getConstraint() {
        return this.constraint;
    }
}
