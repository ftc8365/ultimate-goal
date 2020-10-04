package org.firstinspires.ftc.teamcode.ultimategoal.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ultimategoal.robot.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.Trajectory;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous2", group="Autonomous")
//@Disabled
public class Autonomous2 extends LinearOpMode {

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

        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        int initRight   = robot.getRightOdometryPosition();
        int initLeft    = robot.getLeftOdometryPosition();
        int initCenter  = robot.getCenterOdometryPosition();

        while (inInitializationState()) {
            telemetry.addData("", "------------------------------");

            robot.clearBulkCache();
            int rightPos    = robot.getRightOdometryPosition();
            int leftPos     = robot.getLeftOdometryPosition();
            int centerPos   = robot.getCenterOdometryPosition();
            int diff        = (rightPos - initRight ) - ( leftPos - initLeft );

            double heading =  -180 * diff / ( robot.ODEMOTRY_WHEEL_DIAMETER * Math.PI * robot.ODOMETRY_WHEEL_TICKS_PER_INCH ) ;

            if (heading < 0)
                heading = 360 + heading;

            telemetry.addData("init_right", initRight);
            telemetry.addData("init_left",  initLeft);
            telemetry.addData("curr_right", rightPos);
            telemetry.addData("curr_left",  leftPos);
            telemetry.addData("diff",       diff);

            telemetry.addData("heading",    heading );

            telemetry.addData("gyro", robot.getCurrentPositionInDegrees());


            telemetry.addData("ctr", centerPos);

            telemetry.addData( "rate/sec",  ++count / timer.seconds() );

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////

        robot.resetAutonomousTimer();

        robot.shutdown();
    }

}