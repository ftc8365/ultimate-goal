package org.firstinspires.ftc.teamcode.ultimategoal.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.robot.*;

@TeleOp(name="DriverControl", group="TeleOp")
//@Disabled

public class DriverControl extends LinearOpMode {
    Robot robot = new Robot(this);

    double DRIVE_NORMAL_POWER_RATIO = 0.50;
//    double DRIVE_NORMAL_POWER_RATIO = 0.60;
    double DRIVE_LOW_POWER_RATIO    = 0.30;
    double TURN_NORMAL_POWER_RATIO  = 0.75;
//    double TURN_NORMAL_POWER_RATIO  = 0.50;
    double TURN_LOW_POWER_RATIO     = 0.25;

    boolean driveNormalMode = true;

    @Override
    public void runOpMode() {
        robot.getDriveTrain().init();
        robot.getGrabber().init();
        robot.getIntake().init();
        robot.getShooter().init();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("", "------------------------------");
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        robot.getShooter().stopPoker();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            operateDriveTrain();
            operateControls();

            telemetry.addData("Intake Velocity", robot.getIntake().getIntakeVelocity());
            telemetry.addData("Intake Position", robot.getIntake().getIntakePosition());
            telemetry.addData("Intake Lift Velocity", robot.getIntake().getIntakeLiftVelocity());
            telemetry.addData("Intake Lift Position", robot.getIntake().getIntakeLiftPosition());
            telemetry.addData("Shooter Velocity", robot.getShooter().getVelocity());

            telemetry.update();
        }

        robot.getShooter().stop();
        robot.getDriveTrain().stop();
    }


    //////////////////////////////////////////////////////////////
    //
    // dpad_left     : Opens grabber
    // dpad_right    : Closes grabber
    // dpad_up       : Extends Arm Up & Backward
    // dpad_down     : Retracts Arm Down & Forward
    //
    // right_trigger : intake
    // left_trigger  : outtake
    // left_bumper   : stop poker
    // right_bumper  : push poker
    //
    // y             : Raises basket + Shooter State 1
    // b             : turns Shooter off
    // x             : bursts poker
    // a             :
    //
    //////////////////////////////////////////////////////////////

    void operateControls() {
        if (gamepad1.dpad_left) {
            robot.getGrabber().openGrabber();
        } else if (gamepad1.dpad_right) {
            robot.getGrabber().closeGrabber();
        } else if (gamepad1.dpad_up) {
            robot.getGrabber().armUp();
        } else if (gamepad1.dpad_down) {
            robot.getGrabber().armDown();
        }

        if ((gamepad1.right_trigger > 0) || (gamepad2.right_trigger > 0)) {
            if (robot.getIntake().isBasketUp()) {
                robot.getShooter().stopPoker();
                robot.getShooter().setState(Shooter.ShooterState.SHOOTER_STATE_2);
                robot.getIntake().lowerBasket();
            }
            robot.getIntake().intake();

        } else if (gamepad1.left_trigger > 0) {
            robot.getIntake().outake();
        } else {
            robot.getIntake().stop();
        }

        if (gamepad1.y) {
            robot.getShooter().stopPoker();
            robot.getIntake().liftBasket();
            robot.getShooter().setState(Shooter.ShooterState.SHOOTER_STATE_1);
        } else if (gamepad1.b) {
            robot.getShooter().stopPoker();
            robot.getIntake().lowerBasket();
            robot.getShooter().setState(Shooter.ShooterState.SHOOTER_STATE_2);
        }

        if (gamepad1.right_bumper ) {
            robot.getShooter().pushPoker();
        }
        else if (gamepad1.left_bumper){
            robot.getShooter().stopPoker();
        }
        else if (gamepad1.x) {
            robot.getShooter().burstPoker();
        }
    }


    /////////////////////////////////////////////////////////////////////////////////
    // operateDriveTrain
    /////////////////////////////////////////////////////////////////////////////////
    void operateDriveTrain() {

        double drivePowerRatio = this.DRIVE_NORMAL_POWER_RATIO;
        double turnPowerRatio  = this.TURN_NORMAL_POWER_RATIO;

        if (gamepad1.left_stick_button) {
            drivePowerRatio = this.DRIVE_LOW_POWER_RATIO;
        }

        if (gamepad1.right_stick_button){
            turnPowerRatio  = this.TURN_LOW_POWER_RATIO;
        }

        double driveStickXValue = gamepad1.left_stick_x;
        double driveStickYValue = gamepad1.left_stick_y;

        double turnStickXValue = gamepad1.right_stick_x;
        double turnStickYValue = gamepad1.right_stick_y;

        int joystickPosition = getJoystickPosition(driveStickXValue, driveStickYValue);

        double value = Math.max( Math.abs(driveStickYValue), Math.abs(driveStickXValue));

        double motorFRPower = 0;
        double motorFLPower = 0;
        double motorBRPower = 0;
        double motorBLPower = 0;

        switch (joystickPosition) {
            case 1:
                motorFRPower = 1 * value;
                motorFLPower = 1 * value;
                motorBRPower = 1 * value;
                motorBLPower = 1 * value;
                break;

            case 2:
                motorFRPower = 0 * value;
                motorFLPower = 1 * value;
                motorBRPower = 1 * value;
                motorBLPower = 0 * value;
                break;

            case 3:
                motorFRPower = -1 * value;
                motorFLPower =  1 * value;
                motorBRPower =  1 * value;
                motorBLPower = -1 * value;
                break;

            case 4:
                motorFRPower = -1 * value;
                motorFLPower = 0 * value;
                motorBRPower = 0 * value;
                motorBLPower = -1 * value;
                break;
            case 5:
                motorFRPower = -1 * value;
                motorFLPower = -1 * value;
                motorBRPower = -1 * value;
                motorBLPower = -1 * value;
                break;
            case 6:
                motorFRPower =  0 * value;
                motorFLPower = -1 * value;
                motorBRPower = -1 * value;
                motorBLPower =  0 * value;
                break;
            case 7:
                motorFRPower =  1 * value;
                motorFLPower = -1 * value;
                motorBRPower = -1 * value;
                motorBLPower =  1 * value;
                break;
            case 8:
                motorFRPower = 1 * value;
                motorFLPower = 0 * value;
                motorBRPower = 0 * value;
                motorBLPower = 1 * value;
                break;
            case 0:
                motorFRPower = 0;
                motorFLPower = 0;
                motorBRPower = 0;
                motorBLPower = 0;
                break;
        }

        motorFRPower = motorFRPower * drivePowerRatio;
        motorFLPower = motorFLPower * drivePowerRatio;
        motorBRPower = motorBRPower * drivePowerRatio;
        motorBLPower = motorBLPower * drivePowerRatio;

        ////////////////////////////////////////////////////////////
        // if the robot is moving forward or backward (pos 1 or 5)
        // and is turning, the robot will curve
        // otherwise, the robot will just turn
        ////////////////////////////////////////////////////////////
        if (Math.abs(turnStickXValue) > 0.1) {

            switch (joystickPosition) {
                case 1:
                case 5:
                    motorFRPower = motorFRPower - (turnStickXValue * motorFRPower * 0.5 );
                    motorFLPower = motorFLPower + (turnStickXValue * motorFRPower * 0.5 );
                    motorBRPower = motorBRPower - (turnStickXValue * motorFRPower * 0.5 );
                    motorBLPower = motorBLPower + (turnStickXValue * motorFRPower * 0.5 );
                    break;
                default:
                    motorFRPower = -1 * turnStickXValue * turnPowerRatio;
                    motorFLPower =  1 * turnStickXValue * turnPowerRatio;
                    motorBRPower = -1 * turnStickXValue * turnPowerRatio;
                    motorBLPower =  1 * turnStickXValue * turnPowerRatio;
                    break;
            }
        }

        robot.getDriveTrain().setDriveMotorPower(motorFRPower, motorFLPower, motorBRPower, motorBLPower);
    }

    int getJoystickPosition(double x, double y) {

        if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1)
            return 0;

        int quadrant = 0;
        double angle = Math.atan( Math.abs(x) / Math.abs(y)) * 180 / Math.PI;

        if (x >= 0 & y <= 0)
            quadrant = 1;
        else if (x >= 0 & y >= 0) {
            angle = 180 - angle;
            quadrant = 2;
        }
        else if (x <= 0 & y > 0) {
            angle = 180 + angle;
            quadrant = 3;
        }
        else if (x <= 0 & y <= 0) {
            angle = 360 - angle;
            quadrant = 4;
        }

        return (int)(((angle + 22.5)%360) / 45) + 1;
    }

}
