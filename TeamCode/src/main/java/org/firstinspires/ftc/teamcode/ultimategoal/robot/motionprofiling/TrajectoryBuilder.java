package org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling;

public class TrajectoryBuilder {
    Trajectory trajectory = new Trajectory();

    public TrajectoryBuilder moveForward( double distanceInInches ) {
        trajectory.getMotions().add( new Motion( Motion.Type.MOVE_FORWARD, distanceInInches, -1, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder moveForward( double distanceInInches, Constraint constraint ) {
        trajectory.getMotions().add( new Motion( Motion.Type.MOVE_FORWARD, distanceInInches, -1, constraint ) );
        return this;
    }

    public TrajectoryBuilder moveBackward( double distanceInInches ) {
        trajectory.getMotions().add( new Motion( Motion.Type.MOVE_BACKWARD, distanceInInches, -1, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder moveBackward( double distanceInInches, Constraint constraint ) {
        trajectory.getMotions().add( new Motion( Motion.Type.MOVE_BACKWARD, distanceInInches, -1, constraint ) );
        return this;
    }

    public TrajectoryBuilder strafeLeft( double distanceInInches ) {
        trajectory.getMotions().add( new Motion( Motion.Type.STRAFE_LEFT, distanceInInches, -1, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder strafeLeft( double distanceInInches, Constraint constraint ) {
        trajectory.getMotions().add( new Motion( Motion.Type.STRAFE_LEFT, distanceInInches, -1, constraint ) );
        return this;
    }

    public TrajectoryBuilder strafeRight( double distanceInInches ) {
        trajectory.getMotions().add( new Motion( Motion.Type.STRAFE_RIGHT, distanceInInches, -1, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder strafeRight( double distanceInInches, Constraint constraint ) {
        trajectory.getMotions().add( new Motion( Motion.Type.STRAFE_RIGHT, distanceInInches, -1, constraint ) );
        return this;
    }

    public TrajectoryBuilder turnLeft( int targetHeading ) {
        trajectory.getMotions().add( new Motion( Motion.Type.TURN_LEFT, 0, targetHeading, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder turnLeft( int targetHeading, Constraint constraint ) {
        trajectory.getMotions().add( new Motion( Motion.Type.TURN_LEFT, 0, targetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder turnRight( int targetHeading ) {
        trajectory.getMotions().add( new Motion( Motion.Type.TURN_RIGHT, 0, targetHeading, new Constraint() ) );
        return this;
    }

    public TrajectoryBuilder turnRight( int targetHeading, Constraint constraint ) {
        trajectory.getMotions().add( new Motion( Motion.Type.TURN_RIGHT, 0, targetHeading, constraint ) );
        return this;
    }

    public Trajectory build() {
        trajectory.getMotions().add( new Motion( Motion.Type.STOP, 0, 0, new Constraint() ) );
        return this.trajectory;
    }

}
