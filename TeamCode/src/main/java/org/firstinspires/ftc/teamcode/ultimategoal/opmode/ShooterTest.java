package org.firstinspires.ftc.teamcode.ultimategoal.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.ultimategoal.robot.Robot;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.Shooter;

@TeleOp(name="ShooterTest", group="TeleOp")
//@Disabled

public class ShooterTest extends LinearOpMode {

    Robot robot = new Robot(this);


    @Override
    public void runOpMode() {
        robot.getShooter().init();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("", "------------------------------");
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            operateShooter();
        }

        robot.getShooter().stop();
    }


    void operateShooter() {
        if (gamepad1.y) {
            robot.getShooter().setState(Shooter.ShooterState.SHOOTER_STATE_1);
        } else if (gamepad1.x) {
            robot.getShooter().setState(Shooter.ShooterState.SHOOTER_STATE_2);
        } else if(gamepad1.b) {
            robot.getShooter().setState(Shooter.ShooterState.SHOOTER_OFF);
        }
    }

}
