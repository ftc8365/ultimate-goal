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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private boolean inInitializationState() {
        return (!opModeIsActive() && !isStopRequested());
    }

    @Override
    public void runOpMode() {
        robot.initDriveTrain();
        robot.initComputerVision();

        Trajectory trajectory = robot.trajectoryBuilder()
                .moveForward(20, 0 )
                .moveBackward(20, 0 )
                .turnLeft( 270 )
                .turnRight( 90 )
                .strafeLeft(6,0)
                .strafeRight(6,0)
                .build();

        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        while (inInitializationState()) {
            telemetry.addData("", "------------------------------");

            robot.getComputerVision().detect();

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

        if (opModeIsActive()) {

            robot.followTrajectory(trajectory);
        }

        robot.shutdown();
    }

}