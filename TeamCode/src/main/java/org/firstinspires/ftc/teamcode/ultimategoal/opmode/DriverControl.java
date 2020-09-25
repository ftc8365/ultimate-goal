package org.firstinspires.ftc.teamcode.ultimategoal.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ultimategoal.robot.Robot;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DriverControl", group="TeleOp")
//@Disabled

public class DriverControl extends LinearOpMode {

    Robot robot = new Robot();

    double DRIVE_NORMAL_POWER_RATIO = 1.00;
    double DRIVE_LOW_POWER_RATIO    = 0.30;
    double TURN_NORMAL_POWER_RATIO  = 0.75;
    double TURN_LOW_POWER_RATIO     = 0.25;

    boolean driveNormalMode = true;

    @Override
    public void runOpMode() {
        robot.setOpMode(this);
        robot.setRunningAutonomous(false);
        robot.initDriveTrain();

        int initPos = robot.motorBR.getCurrentPosition();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("", "------------------------------");
            telemetry.addData(">", "Press Play to start");
            telemetry.addData("init", initPos);
            telemetry.addData("curr", robot.motorBR.getCurrentPosition());
            telemetry.addData("diff", robot.motorBR.getCurrentPosition() - initPos);

            telemetry.addData("heading", robot.getCurrentPositionInDegrees());
            telemetry.addData("curheading", robot.getCurrentHeading());

            telemetry.update();
        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            operateDriveTrain();
            telemetry.update();
        }

        robot.stopDriveMotors();
    }

    /////////////////////////////////////////////////////////////////////////////////
    // operateDriveTrain
    /////////////////////////////////////////////////////////////////////////////////
    void operateDriveTrain() {

        double drivePowerRatio = this.DRIVE_NORMAL_POWER_RATIO;
        double turnPowerRatio  = this.TURN_NORMAL_POWER_RATIO;

        if (gamepad1.x){
            driveNormalMode = false;
        }
        else if (gamepad1.y){
            driveNormalMode = true;
        }

        if (driveNormalMode == false){
            drivePowerRatio = this.DRIVE_LOW_POWER_RATIO;
        }

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

        ////////////////////
        // if the robot is moving forward or backward (pos 1 or 5)
        // and is turning, the robot will curve
        // otherwise, the robot will just turn
        ////////////////////
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

        robot.motorFR.setPower(motorFRPower);
        robot.motorFL.setPower(motorFLPower);
        robot.motorBR.setPower(motorBRPower);
        robot.motorBL.setPower(motorBLPower);

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
