package org.firstinspires.ftc.teamcode.ultimategoal.robot;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.Motion;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.Trajectory;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.TrajectoryBuilder;

import java.util.List;

public class Robot {

    //////////////////////////////////////////////////////////////////////
    // Declare robot sub modules
    //////////////////////////////////////////////////////////////////////

    MecanumDriveTrain   driveTrain;
    ComputerVision      computerVision;
    Grabber             grabber;
    Intake              intake;
    Shooter             shooter;

    //////////////////////////////////////////
    // Declare sensors
    //////////////////////////////////////////
    ElapsedTime         autonomusTimer;
    final LinearOpMode  opMode;

    //----------------------------------------------------------------------------------------------------------------------------------
    // Class Methods Starts Here
    //----------------------------------------------------------------------------------------------------------------------------------

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Constructor
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Robot( LinearOpMode opMode ) {
        this.autonomusTimer     = new ElapsedTime();
        this.opMode             = opMode;

        this.driveTrain         = new MecanumDriveTrain(this);
        this.computerVision     = new ComputerVision(this);
        this.grabber            = new Grabber(this);
        this.intake             = new Intake(this);
        this.shooter            = new Shooter(this);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public MecanumDriveTrain getDriveTrain() {
        return this.driveTrain;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public ComputerVision getComputerVision() {
        return this.computerVision;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Grabber getGrabber() {
        return this.grabber;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Intake getIntake() {return this.intake; }
    // resetAutonomousTimer
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public Shooter getShooter() {return this.shooter;}
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    public void resetAutonomousTimer() {
        autonomusTimer.reset();
    }

    boolean continueMotion() {
        final int    AUTONOMOUS_DURATION_MSEC = 29800;

        return (opMode.opModeIsActive()) && (autonomusTimer.milliseconds() < AUTONOMOUS_DURATION_MSEC);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void followTrajectory( Trajectory trajectory ) {
        List< Motion > movements = trajectory.getMotions();

        int targetHeading = 0;

        for (Motion motion : movements) {
            switch (motion.getType()) {
                case MOVE_FORWARD:
                    getDriveTrain().driveForwardTillDistance( motion.getDistance(),
                            motion.getConstraint().getTargetPower(),
                                                    targetHeading,
                                                    motion.getConstraint().getRampDown(),
                                                    motion.getConstraint().getStopMotor());
                    break;
                case MOVE_BACKWARD:
                    getDriveTrain().driveBackwardTillDistance( motion.getDistance(),
                                                                motion.getConstraint().getTargetPower(),
                                                                targetHeading,
                                                                motion.getConstraint().getRampDown(),
                                                                motion.getConstraint().getStopMotor());
                    break;
                case STRAFE_LEFT:
                    getDriveTrain().strafeLeftTillDistance( motion.getDistance(),
                                                            motion.getConstraint().getTargetPower(),
                                                            targetHeading,
                                                            motion.getConstraint().getRampDown(),
                                                            motion.getConstraint().getStopMotor());
                    break;
                case STRAFE_RIGHT:
                    getDriveTrain().strafeRightTillDistance( motion.getDistance(),
                                                    motion.getConstraint().getTargetPower(),
                                                    targetHeading,
                                                    motion.getConstraint().getRampDown(),
                                                    motion.getConstraint().getStopMotor());
                    break;
                case TURN_LEFT:
                    targetHeading = motion.getTargetHeading();
                    getDriveTrain().turnLeftTillDegrees( targetHeading,
                                                        motion.getConstraint().getTargetPower(),
                                                        motion.getConstraint().getRampDown(),
                                                        motion.getConstraint().getStopMotor() );
                    break;
                case TURN_RIGHT:
                    targetHeading = motion.getTargetHeading();
                    getDriveTrain().turnRightTillDegrees( targetHeading,
                                                            motion.getConstraint().getTargetPower(),
                                                            motion.getConstraint().getRampDown(),
                                                            motion.getConstraint().getStopMotor() );
                    break;
                case STOP:
                    getDriveTrain().stopDriveMotors();
                    break;
            }
        }
    }
}
