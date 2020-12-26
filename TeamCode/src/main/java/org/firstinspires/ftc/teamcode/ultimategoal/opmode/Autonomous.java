package org.firstinspires.ftc.teamcode.ultimategoal.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ultimategoal.robot.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.Shooter;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.Trajectory;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Autonomous")
//@Disabled
public class Autonomous extends LinearOpMode {

    //////////////////////////////////////////////////////////////////////
    // Declare OpMode members
    //////////////////////////////////////////////////////////////////////

    ElapsedTime timer = new ElapsedTime();

    Robot robot = new Robot( this );

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private boolean inInitializationState() {
        return (!opModeIsActive() && !isStopRequested());
    }

    @Override
    public void runOpMode() {
        String ringPatten = "";


        // intialize robot submodules
        robot.getDriveTrain().init();
        robot.getGrabber().init();
        robot.getShooter().init();
        robot.getIntake().init();
        robot.getComputerVision().init();

        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        // Setting drive train to float unlocks wheels to allow movemment and to confirm odometry readings are working of robot after init
        robot.getDriveTrain().setDriveTrainZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT );
        robot.getComputerVision().activate();;
        robot.getIntake().liftBasket();
        robot.getGrabber().closeGrabber();

        while (inInitializationState()) {
            robot.getDriveTrain().clearBulkCache();

            ringPatten = robot.getComputerVision().detect();

            int leftPosition = robot.getDriveTrain().getLeftOdometryPosition();
            int rightPosition = robot.getDriveTrain().getRightOdometryPosition();

            telemetry.addData("ring pattern",   ringPatten);
            telemetry.addData("", "------------------------------");
            telemetry.addData("left",   leftPosition);
            telemetry.addData("right",  rightPosition);

            telemetry.addData("left Chg",   leftPosition - robot.getDriveTrain().getInitLeftOdometryPosition());
            telemetry.addData("right Chg",  rightPosition - robot.getDriveTrain().getInitRightOdometryPosition());

            telemetry.addData("PosInDegrees",   String.format("%.2f",robot.getDriveTrain().getCurrentPositionInDegreesUsingOdometry(rightPosition, leftPosition) ));

            telemetry.addData("", "------------------------------");
            telemetry.addData( "rate/sec",  String.format("%.2f", ++count / timer.seconds()) );

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        robot.getComputerVision().stop();

        if (!opModeIsActive() || isStopRequested())
            return;

        robot.resetAutonomousTimer();
        robot.getDriveTrain().setDriveTrainZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////
        robot.getShooter().setState(Shooter.ShooterState.SHOOTER_STATE_1);
        robot.getGrabber().armUp();
        robot.getShooter().stopPoker();

        if (ringPatten.equals("Quad")) {
            runAutoZoneC();
        }else if (ringPatten.equals("Single")) {
             runAutoZoneB();
        }else {
             runAutoZoneA();
        }
        robot.getDriveTrain().stop();
        robot.getShooter().stop();
        robot.getIntake().stop();

    }

    public void dropWobbleGoal() {
         robot.getGrabber().armDown();
         sleep(350);
         robot.getGrabber().openGrabber();
         sleep(750);
    }
    public void grabWobbleGoal() {
        robot.getGrabber().closeGrabber();
        sleep(750);
        robot.getGrabber().armUp();
    }

    public void runAutoZoneA() {
        Trajectory trajectoryZoneA1 = robot.trajectoryBuilder()
                .setDefaultTargetPower( 0.40 )
                .moveForward( 60 )
                .turnLeft( 340 )
                .stop()
                .turnRight( 60 )
                .moveForward( 2 )
                .stop()
                .moveBackward( 36 )
                .turnRight( 180 )
                .moveForward( 24 )
                .stop()
                .moveBackward( 21 )
                .turnLeft( 60 )
                .moveForward( 33 )
                .build();

        robot.followTrajectory((trajectoryZoneA1));

        robot.getShooter().burstPoker();

        robot.resumeTrajectory((trajectoryZoneA1));

        dropWobbleGoal();

        robot.resumeTrajectory((trajectoryZoneA1));

        grabWobbleGoal();

        robot.resumeTrajectory((trajectoryZoneA1));

        dropWobbleGoal();
    }

    public void runAutoZoneB(){
        Trajectory trajectoryZoneB = robot.trajectoryBuilder()
                .setDefaultTargetPower( 0.40 )
                .moveForward( 60 )
                .turnLeft( 340 )
                .stop()
                .turnRight( 0 )
                .moveForward( 24 )
                .stop()
                .moveBackward( 24 )
                .turnRight( 60 )
                .moveBackward( 34 )
                .turnRight( 180 )
                .moveForward( 24 )
                .stop()
                .moveBackward( 24 )
                .turnLeft( 60 )
                .moveForward( 31 )
                .turnLeft( 0 )
                .moveForward( 24 )
                .build();

        robot.followTrajectory((trajectoryZoneB));

        robot.getShooter().burstPoker();

        robot.resumeTrajectory((trajectoryZoneB));

        dropWobbleGoal();

        robot.resumeTrajectory((trajectoryZoneB));

        grabWobbleGoal();

        robot.resumeTrajectory((trajectoryZoneB));
    }

    public void runAutoZoneC(){
        Trajectory trajectoryZoneC = robot.trajectoryBuilder()
                .setDefaultTargetPower( 0.40 )
                .moveForward( 60 )
                .turnLeft( 340 )
                .stop()
                .turnRight( 0 )
                .moveForward( 24 )
                .turnRight( 90 )
                .stop()
                .moveBackward( 24 )
                .turnRight( 180 )
                .moveForward( 24 )
                .stop()
                .moveBackward( 21 )
                .turnLeft( 60 )
                .moveForward( 33 )
                .build();

        robot.followTrajectory((trajectoryZoneC));

        robot.getShooter().burstPoker();

        robot.resumeTrajectory((trajectoryZoneC));

    }
}