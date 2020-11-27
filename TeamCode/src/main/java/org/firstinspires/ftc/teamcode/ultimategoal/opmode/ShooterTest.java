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
        robot.getShooter().initShooterTest();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("", "------------------------------");
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            operateShooter();

            telemetry.addData("Shooter Power",    String.format("%.2f", robot.getShooter().getPower()));
            telemetry.addData("Shooter Velocity", String.format("%.2f", robot.getShooter().getVelocity()));
            telemetry.update();
        }

        robot.getShooter().stop();
    }



    void operateShooter() {
        if (gamepad1.dpad_left) {
            robot.getShooter().setState(Shooter.ShooterState.SHOOTER_STATE_1);
        } else if (gamepad1.dpad_up) {
            robot.getShooter().setState(Shooter.ShooterState.SHOOTER_STATE_2);
        } else if(gamepad1.dpad_right){
            robot.getShooter().setState(Shooter.ShooterState.SHOOTER_STATE_3);
        } else if(gamepad1.dpad_down){
            robot.getShooter().setState(Shooter.ShooterState.SHOOTER_STATE_4);
        } else if(gamepad1.b) {
            robot.getShooter().setState(Shooter.ShooterState.SHOOTER_OFF);
        }

        if (gamepad1.x) {
            robot.getShooter().pushPokerTest();
        } else if(gamepad1.y){
            robot.getShooter().stopPokerTest();
        }

        if (gamepad1.right_bumper){
            robot.getShooter().burstPoker();
        }

    }

}
