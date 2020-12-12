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

        Trajectory trajectoryzoneA1 = robot.trajectoryBuilder()
                .moveForward( 60 )
                .turnLeft(345)
                .build();


        Trajectory trajectoryzoneA3 = robot.trajectoryBuilder()
                .turnRight(60)
                .moveForward(2)
                .build();

        Trajectory trajectoryzoneA4 = robot.trajectoryBuilder()
                .turnRight(60)
                .moveBackward(36)
                .turnRight(180)
                .moveForward(21)
                .build();

        Trajectory trajectoryzoneA5 = robot.trajectoryBuilder()
                .turnRight(180)
                .moveBackward(20)
                .turnLeft(60)
                .moveForward(33)
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
//        robot.getComputerVision().activate();;
        robot.getIntake().liftBasket();
        robot.getGrabber().closeGrabber();

        while (inInitializationState()) {
            robot.getDriveTrain().clearBulkCache();

//            ringPatten = robot.getComputerVision().detect();

            telemetry.addData("ring pattern",   ringPatten);
            telemetry.addData("", "------------------------------");
            telemetry.addData("left",   robot.getDriveTrain().getLeftOdometryPosition());
            telemetry.addData("right",  robot.getDriveTrain().getRightOdometryPosition());

            telemetry.addData("left Chg",   robot.getDriveTrain().getLeftOdometryPosition() - robot.getDriveTrain().getInitLeftOdometryPosition());
            telemetry.addData("right Chg",  robot.getDriveTrain().getRightOdometryPosition() - robot.getDriveTrain().getInitRightOdometryPosition());

            telemetry.addData("odom",   String.format("%.2f",robot.getDriveTrain().getCurrentPositionInDegreesUsingOdometry() ));
            telemetry.addData("gyro",   String.format("%.2f",robot.getDriveTrain().getCurrentPositionInDegreesUsingGyro()));

            telemetry.addData("", "------------------------------");
            telemetry.addData( "rate/sec",  String.format("%.2f", ++count / timer.seconds()) );

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        robot.getComputerVision().shutdown();

        if (!opModeIsActive() || isStopRequested())
            return;

        robot.getDriveTrain().setDriveTrainZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        robot.resetAutonomousTimer();

        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////
        robot.getShooter().setState(Shooter.ShooterState.SHOOTER_STATE_1);
        robot.getGrabber().armUp();
        robot.getShooter().stopPoker();
        robot.followTrajectory((trajectoryzoneA1));

        robot.getShooter().burstPoker();

        robot.followTrajectory((trajectoryzoneA3));
        robot.getGrabber().armDown();
        sleep(350);
        robot.getGrabber().openGrabber();
        sleep(1000);

        robot.followTrajectory((trajectoryzoneA4));
        robot.getGrabber().closeGrabber();
        sleep(1000);
        robot.getGrabber().armUp();

        robot.followTrajectory((trajectoryzoneA5));
        robot.getGrabber().armDown();
        sleep(350);
        robot.getGrabber().openGrabber();
        sleep(1000);



 /*       robot.getGrabber().openGrabber();
        sleep(1000);
        robot.getGrabber().armDown();
        sleep(1000);
        robot.getGrabber().closeGrabber();
        sleep(1000);
        robot.getGrabber().armUp();
        sleep(1000);

        robot.followTrajectory(trajectoryzoneA1);

        robot.getGrabber().armDown();
        sleep(1000);
        robot.getGrabber().openGrabber();
        sleep(1000);
        robot.getGrabber().armUp();
        sleep(1000);

        robot.followTrajectory((trajectoryzoneA2));

        robot.getGrabber().openGrabber();
        sleep(1000);
        robot.getGrabber().armDown();
        sleep(1000);
        robot.getGrabber().closeGrabber();
        sleep(1000);
        robot.getGrabber().armUp();
        sleep(1000);

        robot.followTrajectory((trajectoryzoneA3));

        robot.getGrabber().armDown();
        sleep(1000);
        robot.getGrabber().openGrabber();
        sleep(1000);
        robot.getGrabber().armUp();
        sleep(1000);
        
*/
    }

}