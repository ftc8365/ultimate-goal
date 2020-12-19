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

        robot.getDriveTrain().init();
        robot.getGrabber().init();
        robot.getShooter().init();
        robot.getIntake().init();
        robot.getComputerVision().init();

        Trajectory trajectoryZoneA1 = robot.trajectoryBuilder()
                .setDefaultTargetPower( 0.40 )
                .moveForward( 60 )
                .turnLeft( 340 )
                .stop()
                .turnRight( 60 )
                .moveForward( 24 )
                .stop()
                .moveBackward( 36 )
                .turnRight( 180 )
                .moveForward( 23 )
                .stop()
                .moveBackward( 21 )
                .turnLeft( 60 )
                .moveForward( 33 )
                .build();

/*


        Trajectory trajectoryzoneA2x = robot.trajectoryBuilder()
                .turnLeft( 270 )
                .moveForward( 18 )
                .turnLeft( 180 )
                .moveForward( 35 )
                .build();

        Trajectory trajectoryzoneA3x = robot.trajectoryBuilder()
                .turnLeft( 0 )
                .moveForward( 38 )
                .turnRight( 90 )
                .moveForward( 14 )
                .build();

        Trajectory trajectoryzoneA4x = robot.trajectoryBuilder()
                .moveForward( 54 )
                .turnLeft( 270 )
                .moveForward( 28 )
                .turnLeft( 180 )
                .moveForward( 28 )
                .turnLeft( 0 )
                .moveForward( 28 )
                .turnRight( 90 )
                .moveForward( 24 )
                .build();

        Trajectory trajectory3 = robot.trajectoryBuilder()
                .moveForward( 54 )
                .turnLeft(270)
                .moveForward( 28 )
                .turnLeft(180)
                .moveForward( 28 )
                .turnLeft(0)
                .moveForward( 48 )
                .build();

        Trajectory trajectory4 = robot.trajectoryBuilder()
                .moveForward( 54 )
                .turnLeft(270)
                .moveForward( 28 )
                .turnLeft(180)
                .moveForward( 28 )
                .turnLeft(0)
                .moveForward( 96 )
                .turnRight(90)
                .build();

*/

        ElapsedTime timer = new ElapsedTime();
        int count = 0;

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

            telemetry.addData("odom",   String.format("%.2f",robot.getDriveTrain().getCurrentPositionInDegreesUsingOdometry(rightPosition, leftPosition) ));

            telemetry.addData("", "------------------------------");
            telemetry.addData( "rate/sec",  String.format("%.2f", ++count / timer.seconds()) );

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        //public void dropWobbleGoal() {
           // robot.getGrabber().armDown();
        //    sleep(350);
        //    robot.getGrabber().openGrabber();
          //  sleep(1000);
       // }
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

        // TODO : Add logic to determine ring pattern
        //        Put all the code between BEGIN mission and END mission in a method runAutoZoneA()
        //        Add logic to execute different zone
        // eg,
        //        if (ringPattern.equals("quad"))
        //           runAutoZoneA();

        // BEGIN mission
        robot.followTrajectory((trajectoryZoneA1));

        robot.getShooter().burstPoker();

        robot.resumeTrajectory((trajectoryZoneA1));

        robot.getGrabber().armDown();
        sleep(350);
        robot.getGrabber().openGrabber();
        sleep(1000);
            // TODO : calcuate optimal time to wait for the grabber to open (hint, goBilda Torque servo)

        robot.resumeTrajectory((trajectoryZoneA1));

        // TODO : Create grabWobbleGoal() method so it can be reused
        robot.getGrabber().closeGrabber();
        sleep(1000);
        robot.getGrabber().armUp();

        robot.resumeTrajectory((trajectoryZoneA1));

        // TODO : Update methold to use dropWobbleGoal()
        robot.getGrabber().armDown();
        sleep(350);
        robot.getGrabber().openGrabber();
        sleep(1000);


        // END mission

        robot.getDriveTrain().stop();
        robot.getShooter().stop();
        robot.getIntake().stop();

    }

}