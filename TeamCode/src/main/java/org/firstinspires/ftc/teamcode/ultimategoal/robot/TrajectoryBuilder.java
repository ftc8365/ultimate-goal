package org.firstinspires.ftc.teamcode.ultimategoal.robot;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryBuilder {
    Trajectory trajectory = new Trajectory();

    public TrajectoryBuilder moveForward( double distanceInInches ) {
        trajectory.getMovements().add( new Movement( Movement.Type.MOVE_FORWARD, distanceInInches, -1, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder moveForward( double distanceInInches, Constraint constraint ) {
        trajectory.getMovements().add( new Movement( Movement.Type.MOVE_FORWARD, distanceInInches, -1, constraint ) );
        return this;
    }

    public TrajectoryBuilder moveBackward( double distanceInInches ) {
        trajectory.getMovements().add( new Movement( Movement.Type.MOVE_BACKWARD, distanceInInches, -1, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder moveBackward( double distanceInInches, Constraint constraint ) {
        trajectory.getMovements().add( new Movement( Movement.Type.MOVE_BACKWARD, distanceInInches, -1, constraint ) );
        return this;
    }

    public TrajectoryBuilder strafeLeft( double distanceInInches ) {
        trajectory.getMovements().add( new Movement( Movement.Type.STRAFE_LEFT, distanceInInches, -1, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder strafeLeft( double distanceInInches, Constraint constraint ) {
        trajectory.getMovements().add( new Movement( Movement.Type.STRAFE_LEFT, distanceInInches, -1, constraint ) );
        return this;
    }

    public TrajectoryBuilder strafeRight( double distanceInInches ) {
        trajectory.getMovements().add( new Movement( Movement.Type.STRAFE_RIGHT, distanceInInches, -1, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder strafeRight( double distanceInInches, Constraint constraint ) {
        trajectory.getMovements().add( new Movement( Movement.Type.STRAFE_RIGHT, distanceInInches, -1, constraint ) );
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
