package org.firstinspires.ftc.teamcode.ultimategoal.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ultimategoal.robot.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.Shooter;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.Trajectory;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous Odom", group="Autonomous")
//@Disabled
public class AutonomousOdom extends LinearOpMode {

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
                .setDefaultTargetPower( 0.50 )
                .turnRight( 90 )
                .turnLeft( 0 )
                .build();

        ElapsedTime timer = new ElapsedTime();
        int count = 0;

        robot.getDriveTrain().setDriveTrainZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT );
//        robot.getComputerVision().activate();;

        while (inInitializationState()) {
            robot.getDriveTrain().clearBulkCache();

//            ringPatten = robot.getComputerVision().detect();

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

        robot.getComputerVision().stop();

        if (!opModeIsActive() || isStopRequested())
            return;

        robot.resetAutonomousTimer();
        robot.getDriveTrain().setDriveTrainZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE );

        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////
        robot.followTrajectory((trajectoryZoneA1));

        robot.getDriveTrain().stop();
        robot.getShooter().stop();
        robot.getIntake().stop();

        timer.reset();
        count = 0;

        while (opModeIsActive()) {
            telemetry.addData("odom", String.format("%.2f", robot.getDriveTrain().getCurrentPositionInDegreesUsingOdometry()));
            telemetry.addData("rate/sec", String.format("%.2f", ++count / timer.seconds()));
            telemetry.update();
        }

    }

}