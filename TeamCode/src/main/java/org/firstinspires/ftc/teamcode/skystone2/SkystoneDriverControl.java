/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.skystone2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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

@TeleOp(name="Skystone - DriverControl", group="TeleOp")
@Disabled

public class SkystoneDriverControl extends LinearOpMode {

    SkystoneRobot robot = new SkystoneRobot();

    double DRIVE_NORMAL_POWER_RATIO = 1.00;
    double DRIVE_LOW_POWER_RATIO = 0.30;
    double TURN_NORMAL_POWER_RATIO = 0.75;
    double TURN_LOW_POWER_RATIO = 0.25;

    double liftGrabberPos = 0.0;
    double liftRotatorPos = 0.0;
    boolean useDirectionAware = false;
    boolean driveNormalMode = true;

    enum LiftState {
        LIFT_UP,
        LIFT_DOWN
    }

    enum StoneState {
        OUT_OFF_HOPPER,
        IN_HOPPER
    }

    LiftState liftState = LiftState.LIFT_DOWN;
    StoneState stoneState = StoneState.OUT_OFF_HOPPER;

    ElapsedTime v4blTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.setOpMode(this);
        robot.setRunningAutonomous(false);
        robot.initDriveMotors();
        robot.initIntakeMotors();
        robot.initLiftMotors();
        robot.initLiftServos();
        robot.initFoundationServos();
        robot.initShuttleServos();

        robot.raiseFoundationServos();
        robot.servoCapstone.setPosition(0.50);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("distance", robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("", "------------------------------");
            telemetry.addData(">", "Press Play to start");
            telemetry.update();

        }

        robot.setV4BLState(SkystoneRobot.V4BLState.V4BL_STATE_INTAKE);
        robot.raiseGrabber();
//        robot.setLatchPosition(SkystoneRobot.LatchPosition.LATCH_POSITION_1);
//        robot.servoCamera.setPosition(0.3);
        v4blTime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            operateIntake();
            operateDriveTrain();
            operateLift();
            operateFoundation();
            operateTape();

            telemetry.addData("MotorTape Pos", (double)robot.motorTape.getCurrentPosition());
            telemetry.update();
        }

        robot.stopAllMotors();
    }

    /////////////////////////////////////////////////////////////////////////////////
    // operateFoundation
    /////////////////////////////////////////////////////////////////////////////////
    void operateFoundation() {
        if (gamepad1.right_bumper) {
            robot.lowerFoundationServos();
        } else if (gamepad1.left_bumper) {
            robot.raiseFoundationServos();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////
    // operateFoundation
    /////////////////////////////////////////////////////////////////////////////////
    void operateTape() {
        if (gamepad1.dpad_up)
            robot.motorTape.setPower(-0.60);
        else if (gamepad1.dpad_down)
            robot.motorTape.setPower(0.70);
        else
            robot.motorTape.setPower(0.00);
    }

    void lockLiftMotor() {
        robot.motorLiftRight.setPower( (this.liftState == LiftState.LIFT_UP) ? -0.10 : 0.00 );
    }

    /////////////////////////////////////////////////////////////////////////////////
    // operateLift
    /////////////////////////////////////////////////////////////////////////////////
    void operateLift(){

        ///////////////////////////
        // linear slide
        ///////////////////////////
        double y = gamepad2.right_stick_y;
        double power = 0;

        if ( Math.abs(y) > 0.01) {

            // Going Down
            if (y > 0.01) {
                power = 0.10;
            }
            else {
                power = y;
            }

            robot.motorLiftRight.setPower( power );

            this.liftState = (power < -0.01) ? LiftState.LIFT_UP : LiftState.LIFT_DOWN;
        }
        else {
            lockLiftMotor();
        }


        ///////////////////////////
        // GRABBER
        ///////////////////////////
        if (gamepad2.left_trigger > 0) {
            robot.lowerGrabber();
        } else if (gamepad2.right_trigger > 0) {
            robot.raiseGrabber();
        }


        ///////////////////////////
        // CAPSTONE
        ///////////////////////////
        if (gamepad2.y) {
            robot.servoCapstone.setPosition(0.0);
        }

        ///////////////////////////
        // V4BL
        ///////////////////////////
        if (gamepad2.dpad_left) {
            robot.stopDriveMotors();
            lockLiftMotor();

            if (robot.isV4BLStateIntake2Foundation())
                robot.setV4BLState( SkystoneRobot.V4BLState.V4BL_STATE_INTAKE);

            stoneState = StoneState.OUT_OFF_HOPPER;
        }
        else if (gamepad2.dpad_right) {
            robot.stopDriveMotors();
            lockLiftMotor();

            if (v4blTime.milliseconds() > 250) {

                if (robot.isV4BLStateTop2Foundation())
                    robot.setV4BLState(SkystoneRobot.V4BLState.V4BL_STATE_FOUNDATION);
                else if (robot.isV4BLStateStone2Top())
                    robot.setV4BLState(SkystoneRobot.V4BLState.V4BL_STATE_TOP);

                v4blTime.reset();
            }

            stoneState = StoneState.OUT_OFF_HOPPER;
        }
        else if (gamepad2.dpad_down) {
            robot.stopDriveMotors();
            lockLiftMotor();

            if (robot.isV4BLState( SkystoneRobot.V4BLState.V4BL_STATE_INTAKE ) ) {
                robot.setV4BLState( SkystoneRobot.V4BLState.V4BL_STATE_STONE);
                robot.lowerGrabber();
            } else if (robot.isV4BLState( SkystoneRobot.V4BLState.V4BL_STATE_STONE ) && robot.getV4BLServoPosition() > SkystoneRobot.V4BLState.V4BL_STATE_STONE.servoPos ) {
                robot.setV4BLState( SkystoneRobot.V4BLState.V4BL_STATE_STONE);
                robot.lowerGrabber();
            }
//            else if (robot.isV4BLState( SkystoneRobot.V4BLState.V4BL_STATE_FOUNDATION ))
//            {
//                robot.offsetV4BLPosition(0.05 , 10);
//            }
        }
        else if (gamepad2.dpad_up) {
            robot.stopDriveMotors();
            lockLiftMotor();

            if (robot.isV4BLState( SkystoneRobot.V4BLState.V4BL_STATE_STONE ) && stoneState == StoneState.IN_HOPPER) {
                robot.raiseGrabber();
                robot.setV4BLState(SkystoneRobot.V4BLState.V4BL_STATE_INTAKE);
            }
        }
        else if (Math.abs(gamepad2.left_stick_x) > 0.10) {

            if (v4blTime.milliseconds() > 25) {

                double position = robot.getV4BLServoPosition();

                if (gamepad2.left_stick_x > 0) {
                    position += 0.05;
                } else {
                    position -= 0.05;
                }
                robot.setV4BLPosition(position);
                v4blTime.reset();
            }
        }

//        telemetry.addData("StoneState", stoneState);
//        telemetry.addData("V4BLState", robot.getV4BLState());
//        telemetry.addData("V4BLPos", robot.getV4BLServoPosition());
    }

    /////////////////////////////////////////////////////////////////////////////////
    // operateIntake
    /////////////////////////////////////////////////////////////////////////////////
    void operateIntake() {

        if (stoneState == StoneState.OUT_OFF_HOPPER && robot.stoneDetected()) {
            robot.grabStone();
            stoneState = StoneState.IN_HOPPER;
            v4blTime.reset();
        }

        //////////////////////////////////////////////////////
        // RIGHT TRIGGER ON         => Intake In
        // LEFT  TRIGGER ON         => Intake Out
        //////////////////////////////////////////////////////
        if ( gamepad1.left_trigger > 0 ) {
            robot.turnIntakeOn( SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_OUT );
        } else if ( gamepad1.right_trigger > 0 ) {
            robot.turnIntakeOn( SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_IN );
        } else {
            robot.turnIntakeoff();
        }

    }

    void operateDriveTrain() {

        double drivePowerRatio = this.DRIVE_NORMAL_POWER_RATIO;
        double turnPowerRatio  = this.TURN_NORMAL_POWER_RATIO;

        ///////////////////////////////////
        // Shuttle Bot Mode
        ///////////////////////////////////
        if(gamepad1.dpad_right){
            robot.lowerServoShuttle();
        }
        else if (gamepad1.dpad_left){
            robot.raiseServoShuttle();
        }

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

        int joystickPosition = 0;

        joystickPosition = getJoystickPosition(driveStickXValue, driveStickYValue);

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

//        telemetry.addData("x", driveStickXValue);
//        telemetry.addData("y", driveStickYValue);
//        telemetry.addData("heading", robot.getCurrentHeading());
//        telemetry.addData("joystick-pos", joystickPosition );
//        telemetry.addData("Direction Aware", this.useDirectionAware);
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


    int getDirectionAwareJoystickPosition(double x, double y)
    {
        int position = getJoystickPosition(x,y);

        if (position == 0)
            return  0;

        double degrees = robot.getCurrentPositionInDegrees();

        int offset = 0;

        if (degrees >= 0 && degrees <= 22.5)
            offset = 0;
        else if (degrees > 22.5 && degrees <= 67.5)
            offset = -1;
        else if (degrees > 67.5 && degrees <= 112.5)
            offset = -2;
        else if (degrees > 112.5 && degrees <= 157.5)
            offset = -3;
        else if (degrees > 157.5 && degrees <= 202.5)
            offset = -4;
        else if (degrees > 202.5 && degrees <= 247.5)
            offset = -5;
        else if (degrees > 247.5 && degrees <= 292.5)
            offset = -6;
        else if (degrees > 337.5 && degrees <= 337.5)
            offset = -7;
        else
            offset = 0;


        position += offset;
        if (position <= 0)
            position += 8;

        return position;
    }

}


