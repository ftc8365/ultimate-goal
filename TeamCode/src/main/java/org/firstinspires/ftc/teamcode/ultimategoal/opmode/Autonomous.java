package org.firstinspires.ftc.teamcode.ultimategoal.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ultimategoal.robot.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.Trajectory;

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

        robot.initDriveTrain();
        robot.initComputerVision();

        Trajectory trajectory = robot.trajectoryBuilder()
                .moveForward(54 )
                .turnLeft( 270 )
                .moveForward(28 )
                .turnLeft( 180 )
                .moveForward(28 )
                .turnLeft( 0 )
                .moveForward(28 )
                .turnRight( 90 )
                .moveForward(24 )
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

        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        robot.setDriveTrainZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT );

        while (inInitializationState()) {
            telemetry.addData("", "------------------------------");

            //            robot.getComputerVision().detect();

            robot.clearBulkCache();
            telemetry.addData("ctr",   robot.getCenterOdometryPosition());
            telemetry.addData("odom",   robot.getCurrentPositionInDegreesUsingOdometry() );
            telemetry.addData("gyro",   robot.getCurrentPositionInDegreesUsingGyro());

            telemetry.addData( "rate/sec",  ++count / timer.seconds() );

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////

        robot.setDriveTrainZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );
        robot.resetAutonomousTimer();

        if (opModeIsActive()) {
            robot.followTrajectory(trajectory2);
        }

        robot.clearBulkCache();
/*
        int rightPos = robot.getRightOdometryPosition();
        int leftPos = robot.getLeftOdometryPosition();
        int centerPos = robot.getCenterOdometryPosition();

        telemetry.addData("init_right", rightInit);
        telemetry.addData("init_left", leftInit);
        telemetry.addData("init_ctr", centerInit);

        telemetry.addData("curr_right", rightPos);
        telemetry.addData("curr_left", leftPos);
        telemetry.addData("curr_ctr", centerPos); */

        robot.clearBulkCache();
        telemetry.addData("odom", robot.getCurrentPositionInDegreesUsingOdometry());
        telemetry.addData("gyro", robot.getCurrentPositionInDegreesUsingGyro());
        telemetry.update();

        sleep(1000000);
        robot.shutdown();
    }

}