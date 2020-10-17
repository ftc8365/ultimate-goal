package org.firstinspires.ftc.teamcode.ultimategoal.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ultimategoal.robot.Robot;
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
        robot.getComputerVision().init();

        Trajectory trajectory = robot.trajectoryBuilder()
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

        Trajectory trajectory2 = robot.trajectoryBuilder()
                .moveForward( 6 )
                .turnRight(90)
                .moveForward( 6 )
                .turnRight(180)
                .moveForward( 6 )
                .turnRight(270)
                .moveForward( 6 )
                .turnRight(0)
                .moveForward( 6 )
//                .turnRight(90)
//                .moveForward(18, 90)
//                .strafeLeft(6, 0)
//                .strafeRight(6, 0)
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


        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        robot.getDriveTrain().setDriveTrainZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT );

        while (inInitializationState()) {
            robot.getDriveTrain().clearBulkCache();

            ringPatten = robot.getComputerVision().detect();


            telemetry.addData("ring pattern",   ringPatten);
            telemetry.addData("", "------------------------------");
            telemetry.addData("left",   robot.getDriveTrain().getLeftOdometryPosition());
            telemetry.addData("right",  robot.getDriveTrain().getRightOdometryPosition());
            telemetry.addData("odom",   String.format("%.2f",robot.getDriveTrain().getCurrentPositionInDegreesUsingOdometry() ));
            telemetry.addData("gyro",   String.format("%.2f",robot.getDriveTrain().getCurrentPositionInDegreesUsingGyro()));

            telemetry.addData("", "------------------------------");
            telemetry.addData( "rate/sec",  String.format("%.2f", ++count / timer.seconds()) );

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        robot.getDriveTrain().setDriveTrainZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        robot.resetAutonomousTimer();

        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////

        if (opModeIsActive()) {
            robot.followTrajectory(trajectory2);

            // TODO : 

        }

        robot.getComputerVision().shutdown();
    }

}