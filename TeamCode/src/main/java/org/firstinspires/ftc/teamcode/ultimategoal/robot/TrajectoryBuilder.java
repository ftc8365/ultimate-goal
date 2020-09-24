package org.firstinspires.ftc.teamcode.ultimategoal.robot;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryBuilder {
    Trajectory trajectory = null;
    List< Movement > movements = new ArrayList< Movement >();

    public TrajectoryBuilder forward( double distanceInInches, int targetHeading, Constraint constraint ) {
        movements.add( new Movement( Movement.Type.FORWARD, distanceInInches, targetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder backward( double distanceInInches, int targetHeading, Constraint constraint ) {
        movements.add( new Movement( Movement.Type.BACKWARD, distanceInInches, targetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder strafeLeft( double distanceInInches, int targetHeading, Constraint constraint ) {
        movements.add( new Movement( Movement.Type.STRAFE_LEFT, distanceInInches, targetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder strafeRight( double distanceInInches, int targetHeading, Constraint constraint ) {
        movements.add( new Movement( Movement.Type.STRAFE_RIGHT, distanceInInches, targetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder turnLeft( int targetHeading, Constraint constraint ) {
        movements.add( new Movement( Movement.Type.TURN_LEFT, 0, targetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder turnRight( int targetHeading, Constraint constraint ) {
        movements.add( new Movement( Movement.Type.TURN_RIGHT, 0, targetHeading, constraint ) );
        return this;
    }

    public Trajectory build() {
        movements.add( new Movement( Movement.Type.STOP, 0, 0, new Constraint() ) );
        return trajectory;
    }

}
