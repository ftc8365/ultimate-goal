package org.firstinspires.ftc.teamcode.ultimategoal.robot;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryBuilder {
    Trajectory trajectory = new Trajectory();

    public TrajectoryBuilder moveForward( double distanceInInches, int targetHeading ) {
        trajectory.getMovements().add( new Movement( Movement.Type.MOVE_FORWARD, distanceInInches, targetHeading, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder moveForward( double distanceInInches, int targetHeading, Constraint constraint ) {
        trajectory.getMovements().add( new Movement( Movement.Type.MOVE_FORWARD, distanceInInches, targetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder moveBackward( double distanceInInches, int targetHeading ) {
        trajectory.getMovements().add( new Movement( Movement.Type.MOVE_BACKWARD, distanceInInches, targetHeading, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder moveBackward( double distanceInInches, int targetHeading, Constraint constraint ) {
        trajectory.getMovements().add( new Movement( Movement.Type.MOVE_BACKWARD, distanceInInches, targetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder strafeLeft( double distanceInInches, int targetHeading ) {
        trajectory.getMovements().add( new Movement( Movement.Type.STRAFE_LEFT, distanceInInches, targetHeading, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder strafeLeft( double distanceInInches, int targetHeading, Constraint constraint ) {
        trajectory.getMovements().add( new Movement( Movement.Type.STRAFE_LEFT, distanceInInches, targetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder strafeRight( double distanceInInches, int targetHeading ) {
        trajectory.getMovements().add( new Movement( Movement.Type.STRAFE_RIGHT, distanceInInches, targetHeading, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder strafeRight( double distanceInInches, int targetHeading, Constraint constraint ) {
        trajectory.getMovements().add( new Movement( Movement.Type.STRAFE_RIGHT, distanceInInches, targetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder turnLeft( int targetHeading ) {
        trajectory.getMovements().add( new Movement( Movement.Type.TURN_LEFT, 0, targetHeading, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder turnLeft( int targetHeading, Constraint constraint ) {
        trajectory.getMovements().add( new Movement( Movement.Type.TURN_LEFT, 0, targetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder turnRight( int targetHeading ) {
        trajectory.getMovements().add( new Movement( Movement.Type.TURN_RIGHT, 0, targetHeading, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder turnRight( int targetHeading, Constraint constraint ) {
        trajectory.getMovements().add( new Movement( Movement.Type.TURN_RIGHT, 0, targetHeading, constraint ) );
        return this;
    }

    public Trajectory build() {
        trajectory.getMovements().add( new Movement( Movement.Type.STOP, 0, 0, new Constraint() ) );
        return this.trajectory;
    }

}
