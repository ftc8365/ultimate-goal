package org.firstinspires.ftc.teamcode.ultimategoal.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous Sample", group="Autonomous")
//@Disabled
public class AutonomousSample extends LinearOpMode {

    //////////////////////////////////////////////////////////////////////
    // Declare OpMode members
    //////////////////////////////////////////////////////////////////////

    ElapsedTime timer = new ElapsedTime();

    Robot robot = new Robot( this );

    @Override
    public void runOpMode() {
        robot.initDriveTrain();
        robot.initComputerVision();

        Trajectory trajectory = robot.trajectoryBuilder()
//                .moveForward(20, 0 )
//                .moveBackward(20, 0 )
//                .turnLeft( 270 )
//                .turnRight( 90 )
                .strafeLeft(6,0)
                .strafeRight(6,0)
                .build();

        ElapsedTime timer = new ElapsedTime();
        int count = 0;

//        robot.getInputData();
//        int initPos = robot.getRightOdometryPostion();


        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("", "------------------------------");

            double ms = timer.milliseconds();
//            telemetry.addData( "count", count++ );
            telemetry.addData( "rate",  ++count / ms );

//            robot.getComputerVision().detect();

//            robot.getInputData();
//            int currPos = robot.getRightOdometryPostion();
//            int diff = currPos - initPos;
//            double inches = (double)diff / robot.ODOMETRY_WHEEL_TICKS_PER_INCH; //  ( (double)diff/ 1440 ) * (1.49606 * Math.PI);

//            telemetry.addData("init", initPos);
//                       telemetry.addData("curr", currPos);
//            telemetry.addData("diff", diff);
//            telemetry.addData("dist", inches);
//            telemetry.addData("ctr", robot.getCenterOdometryPosition());

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////

        robot.resetAutonomousTimer();

        robot.followTrajectory(trajectory);

        robot.shutdown();
    }

}