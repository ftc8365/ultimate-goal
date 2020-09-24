package org.firstinspires.ftc.teamcode.ultimategoal.robot;

import org.firstinspires.ftc.robotcore.external.Const;

public class Movement {
    public enum Type {
        FORWARD,
        BACKWARD,
        STRAFE_RIGHT,
        STRAFE_LEFT,
        TURN_RIGHT,
        TURN_LEFT,
        STOP
    };

    private final Type      type;
    private final double    distance;
    private final int       targetHeading;

    private final Constraint constraint;

    Movement( Type type, double distance, int targetHeading, Constraint constraint ) {
        this.type           = type;
        this.distance       = distance;
        this.targetHeading  = targetHeading;
        this.constraint     = constraint;
    }

    Type getType() {
        return this.type;
    }

    double getDistance() {
        return this.distance;
    }

    int getTargetHeading() {
        return this.targetHeading;
    }

    Constraint getConstraint() {
        return this.constraint;
    }
}
