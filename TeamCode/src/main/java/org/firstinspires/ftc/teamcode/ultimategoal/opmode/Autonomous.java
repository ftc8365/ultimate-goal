package org.firstinspires.ftc.teamcode.ultimategoal.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        robot.initDriveTrain();
        robot.initComputerVision();

        Trajectory trajectory = robot.trajectoryBuilder()
                .moveForward(54, 0 )
                .turnLeft( 270 )
                .moveForward(28, 270 )
                .turnLeft( 180 )
                .moveForward(28, 180 )
                .turnLeft( 5 )
                .moveForward(28, 0 )
                .turnRight( 90 )
                .moveForward(24, 90 )
                .build();

        Trajectory trajectory2 = robot.trajectoryBuilder()
//                .moveForward( 12, 0 )
//                .moveBackward(12,0)
                 .turnRight(90)
                .build();

        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        while (inInitializationState()) {
            telemetry.addData("", "------------------------------");

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////
            // TODO : detect() need to return a value (eg, String) that indicates the label of the object detected
            //        - the method should return one of the following:
            //        -   "quad"
            //        -   "Single"
            //        -   "None"
//            robot.getComputerVision().detect();

            robot.clearBulkCache();
            int rightPos = robot.getRightOdometryPosition();
            int leftPos = robot.getLeftOdometryPosition();
            int centerPos = robot.getCenterOdometryPosition();

            telemetry.addData("right", rightPos);
            telemetry.addData("left", leftPos);
            telemetry.addData("ctr", centerPos);
            telemetry.addData("gyro", robot.getCurrentHeading());
            telemetry.addData( "rate/sec",  ++count / timer.seconds() );

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////

        robot.resetAutonomousTimer();
        robot.clearBulkCache();
        int rightInit = robot.getRightOdometryPosition();
        int leftInit = robot.getLeftOdometryPosition();
        int centerInit = robot.getCenterOdometryPosition();

        if (opModeIsActive()) {
            robot.followTrajectory(trajectory2);
        }

        robot.clearBulkCache();
        int rightPos = robot.getRightOdometryPosition();
        int leftPos = robot.getLeftOdometryPosition();
        int centerPos = robot.getCenterOdometryPosition();

        telemetry.addData("init_right", rightInit);
        telemetry.addData("init_left", leftInit);
        telemetry.addData("init_ctr", centerInit);

        telemetry.addData("curr_right", rightPos);
        telemetry.addData("curr_left", leftPos);
        telemetry.addData("curr_ctr", centerPos);

        robot.clearBulkCache();
        telemetry.addData("odom", robot.getCurrentPositionInDegrees());
        telemetry.addData("gyro", robot.getCurrentPositionInDegreesX());

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        sleep(100000);
        robot.shutdown();
    }

}