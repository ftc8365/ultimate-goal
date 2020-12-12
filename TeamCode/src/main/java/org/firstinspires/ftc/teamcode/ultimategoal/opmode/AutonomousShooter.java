package org.firstinspires.ftc.teamcode.ultimategoal.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ultimategoal.robot.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous Test", group="Autonomous")
@Disabled
public class AutonomousShooter extends LinearOpMode {

    //////////////////////////////////////////////////////////////////////
    // Declare OpMode members
    //////////////////////////////////////////////////////////////////////

    ElapsedTime timer = new ElapsedTime();
    Servo servoTest;
    Servo servoTestGobilda;

    Robot robot = new Robot( this );

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private boolean inInitializationState() {
        return (!opModeIsActive() && !isStopRequested());
    }

    @Override
    public void runOpMode() {

        this.servoTest      = hardwareMap.get(Servo.class, "servoTest");
        this.servoTestGobilda      = hardwareMap.get(Servo.class, "servoTest2");

        servoTest.setPosition(0.10);
        servoTestGobilda.setPosition(0.20);
        String ringPatten = "";

        robot.getShooter().init();

        ElapsedTime timer = new ElapsedTime();
        int count = 0;


        while (inInitializationState()) {

            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        if (!opModeIsActive() || isStopRequested())
            return;

        servoTest.setPosition(0.95);
        servoTestGobilda.setPosition(0.7);
        sleep(2000);


/*
        int targetVelocity = 1500;
        robot.getShooter().shooterOn(targetVelocity);


        while  (true) {
            if (timer.seconds() > 20) {
                targetVelocity = 3000;
                robot.getShooter().shooterOn(3000);
            } else if (timer.seconds() > 15) {
                targetVelocity = 1800;
                robot.getShooter().shooterOn(1800);
            } else if (timer.seconds() > 10) {
                targetVelocity = 1700;
                robot.getShooter().shooterOn(1700);
            } else if (timer.seconds() > 5) {
                targetVelocity = 1600;
                robot.getShooter().shooterOn(1600);
            }

            telemetry.addData("seconds", timer.seconds());
            telemetry.addData("target Velocity", targetVelocity);
            telemetry.addData("current Velocity", robot.getShooter().getVelocity());
            telemetry.update();
            sleep(100);
        }
*/
    }

}