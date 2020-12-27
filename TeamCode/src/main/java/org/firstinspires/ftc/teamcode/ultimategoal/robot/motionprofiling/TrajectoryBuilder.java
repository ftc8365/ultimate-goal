package org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling;

public class TrajectoryBuilder {
    Trajectory trajectory           = new Trajectory();
    Constraint defaultConstraint    = new Constraint();

    int currentTargetHeading        = 0;

    public TrajectoryBuilder setDefaultTargetPower( double power ) {
        this.defaultConstraint.setTargetPower( power );
        return this;
    }

    public TrajectoryBuilder stop() {
        trajectory.getMotions().add( new Motion( Motion.Type.STOP, 0, -1, this.defaultConstraint ) );
        return this;
    }

    public TrajectoryBuilder moveForward( double distanceInInches ) {
        trajectory.getMotions().add( new Motion( Motion.Type.MOVE_FORWARD, distanceInInches, currentTargetHeading, this.defaultConstraint ) );
        return this;
    }

    public TrajectoryBuilder moveForward( double distanceInInches, double targetPower ) {
        Constraint constraint = new Constraint();
        constraint.setTargetPower( targetPower );
        trajectory.getMotions().add( new Motion( Motion.Type.MOVE_FORWARD, distanceInInches, currentTargetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder moveForward( double distanceInInches, Constraint constraint ) {
        trajectory.getMotions().add( new Motion( Motion.Type.MOVE_FORWARD, distanceInInches, currentTargetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder moveBackward( double distanceInInches ) {
        trajectory.getMotions().add( new Motion( Motion.Type.MOVE_BACKWARD, distanceInInches, currentTargetHeading, this.defaultConstraint ) );
        return this;
    }

    public TrajectoryBuilder moveBackward( double distanceInInches, Constraint constraint ) {
        trajectory.getMotions().add( new Motion( Motion.Type.MOVE_BACKWARD, distanceInInches, currentTargetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder strafeLeft( double distanceInInches ) {
        trajectory.getMotions().add( new Motion( Motion.Type.STRAFE_LEFT, distanceInInches, currentTargetHeading, this.defaultConstraint ) );
        return this;
    }

    public TrajectoryBuilder strafeLeft( double distanceInInches, Constraint constraint ) {
        trajectory.getMotions().add( new Motion( Motion.Type.STRAFE_LEFT, distanceInInches, currentTargetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder strafeRight( double distanceInInches ) {
        trajectory.getMotions().add( new Motion( Motion.Type.STRAFE_RIGHT, distanceInInches, currentTargetHeading, this.defaultConstraint ) );
        return this;
    }

    public TrajectoryBuilder strafeRight( double distanceInInches, Constraint constraint ) {
        trajectory.getMotions().add( new Motion( Motion.Type.STRAFE_RIGHT, distanceInInches, currentTargetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder turnLeft( int targetHeading ) {
        currentTargetHeading = targetHeading;
        trajectory.getMotions().add( new Motion( Motion.Type.TURN_LEFT, 0, currentTargetHeading, this.defaultConstraint ) );
        return this;
    }

    public TrajectoryBuilder turnLeft( int targetHeading, Constraint constraint ) {
        currentTargetHeading = targetHeading;
        trajectory.getMotions().add( new Motion( Motion.Type.TURN_LEFT, 0, currentTargetHeading, constraint ) );
        return this;
    }

    public TrajectoryBuilder turnRight( int targetHeading ) {
        currentTargetHeading = targetHeading;
        trajectory.getMotions().add( new Motion( Motion.Type.TURN_RIGHT, 0, currentTargetHeading, this.defaultConstraint ) );
        return this;
    }

    public TrajectoryBuilder turnRight( int targetHeading, Constraint constraint ) {
        currentTargetHeading = targetHeading;
        trajectory.getMotions().add( new Motion( Motion.Type.TURN_RIGHT, 0, currentTargetHeading, constraint ) );
        return this;
    }

    public Trajectory build() {
        trajectory.getMotions().add( new Motion( Motion.Type.STOP, 0, -1, this.defaultConstraint ) );
        return this.trajectory;
    }
}
