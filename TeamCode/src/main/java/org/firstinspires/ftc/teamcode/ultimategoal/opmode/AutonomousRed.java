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
        robot.initDriveTrain();

        Trajectory trajectory = robot.trajectoryBuilder( )
                .forward(24, 0, new Constraint())
                .turnLeft(310, new Constraint())
                .forward(12, 310, new Constraint())
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


//        drive.followTrajectory(trajectory);

//        robot.driveForwardTillRotation(1.0, 0.5, 0.5, 90, false, false);


    }


}