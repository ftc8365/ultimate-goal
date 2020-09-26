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

    Robot robot = new Robot(this);

    @Override
    public void runOpMode() {
        robot.initDriveTrain();
        robot.initComputerVision();

        Trajectory trajectory = robot.trajectoryBuilder()
                .moveForward(12, 0 )
//                .moveBackward(6, 0 )
//                .turnLeft( 315 )
//                .moveForward(12, 315 )
//                .turnRight( 45 )
//                .moveBackward(12, 0 )
                .build();

        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("", "------------------------------");

            double ms = timer.milliseconds();

            telemetry.addData( "count", count++ );
            telemetry.addData( "rate",  count / ms );

            robot.getComputerVision().detect();

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