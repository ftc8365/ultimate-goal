package org.firstinspires.ftc.teamcode.ultimategoal.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.ultimategoal.robot.Constraint;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.Trajectory;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto Red", group="Autonomous")
@Disabled
public class AutonomousRed extends LinearOpMode {

    //////////////////////////////////////////////////////////////////////
    // Declare OpMode members
    //////////////////////////////////////////////////////////////////////

    ElapsedTime timer = new ElapsedTime();

    Robot robot = new Robot();

    @Override
    public void runOpMode() {

        robot.setOpMode(this);
        robot.setAllianceMode( Robot.AllianceMode.ALLIANCE_BLUE );
        robot.initDriveTrain();

        Trajectory trajectory = robot.trajectoryBuilder( )
                .moveForward(12, 0 )
                .moveBackward(12, 0 )
                .turnLeft( 315 )
                .moveForward(12, 315 )
                .turnRight( 45 )
                .moveBackward(12, 0 )
                .build();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("", "------------------------------");
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////

        robot.resetAutonomousTimer();

        robot.followTrajectory(trajectory);
    }

}