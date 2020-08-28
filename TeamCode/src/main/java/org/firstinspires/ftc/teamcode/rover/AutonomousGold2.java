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

package org.firstinspires.ftc.teamcode.rover;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


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

@Autonomous(name="Rover - Autonomous Gold Position 2", group="Autonomous")
@Disabled
public class AutonomousGold2 extends LinearOpMode {

    //////////////////////////////////////////////////////////////////////
    // Declare OpMode members
    //////////////////////////////////////////////////////////////////////

    ElapsedTime runtime = new ElapsedTime();

    RoverRobot robot = new RoverRobot();

    ElapsedTime autonomusTimer = new ElapsedTime();



    @Override
    public void runOpMode() {

        robot.initMotors( hardwareMap, true );
        robot.initSensors( hardwareMap );
        robot.initServos( hardwareMap );
        robot.initGyroSensor( hardwareMap );
        robot.initTensorFlowObjectDetection( hardwareMap );

        robot.setPhoneStartingPostion();


        while(!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Gyro Degrees", robot.getCurrentPositionInDegrees());
            telemetry.addData("rangeSensorBottom",  robot.rangeSensorBottom.rawUltrasonic());
            telemetry.addData("rangeSensorFront",   robot.rangeSensorFront.rawUltrasonic());
            telemetry.addData("rangeSensorBack",    robot.rangeSensorBack.rawUltrasonic());

            telemetry.addData("motorFrontRight.currnentPos",    robot.motorFrontRight.getCurrentPosition());
            telemetry.addData("motorCenter.currnentPos",        robot.motorCenter.getCurrentPosition());

            telemetry.addData("",  "------------------------------");
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////
        autonomusTimer.reset();

        runtime.reset();

        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        robot.lockMarker();

        robot.setPhoneScanPosition();

        robot.activateTensorFlowObjectDetection();

        robot.lowerRobot();

        RoverRobot.MineralLocation mineralLocation = RoverRobot.MineralLocation.UNKNOWN;

        runtime.reset();

        while (mineralLocation == RoverRobot.MineralLocation.UNKNOWN)
        {
            mineralLocation = robot.detectMineral(telemetry);

            if ((mineralLocation == RoverRobot.MineralLocation.UNKNOWN) && (runtime.seconds() > 3)) {
                mineralLocation = RoverRobot.MineralLocation.LEFT;
            }

            switch (mineralLocation) {
                case LEFT:
                    leftMineral();
                    break;
                case CENTER:
                    centerMineral();
                    break;
                case RIGHT:
                    rightMineral();
                    break;
            }
        }

        robot.tfod.shutdown();
    }


    void centerMineral()
    {
        robot.driveBackwardTillTime(500, 0.50, telemetry);

        robot.turnLeftTillDegrees(345, 0.50, telemetry);

        robot.driveForwardRotation(0.25, 0.35);

        robot.turnRightTillDegrees(0, 0.50, telemetry);

        robot.driveForwardRotation(2.1, 0.50 );

        robot.turnRightTillDegrees(25, 0.50, telemetry);

        robot.dropMarker();

        robot.driveBackwardRotation(0.1, 0.50 );

        robot.turnLeftTillDegrees(335, 0.50, telemetry);

        robot.driveBackwardRotationAlignWall(3, 0.4, 7, telemetry, autonomusTimer);

        if (autonomusTimer.seconds() <= 24)
            robot.driveBackwardTillTime( 5000, 0.20, telemetry);
    }

    void leftMineral()
    {
        robot.driveBackwardTillTime(500, .5, telemetry);

        robot.turnLeftTillDegrees(330, 0.5, telemetry);

        robot.driveForwardRotation(0.25, 0.35);

        robot.driveForwardRotationTurn(1.75, .35, -.3);

        robot.driveForwardRotation(.35, .35);

        robot.turnRightTillDegrees(25, 0.50, telemetry);

        robot.dropMarker();

        robot.driveBackwardRotation(.35, .35);

        robot.turnRightTillDegrees(305, 0.50, telemetry);

        robot.driveBackwardRotationAlignWall(2.75, 0.4, 7, telemetry, autonomusTimer);

        if (autonomusTimer.seconds() <= 24)
            robot.driveBackwardTillTime( 5000, 0.20, telemetry);
    }

    void rightMineral()
    {
        robot.driveBackwardTillTime(500, .5, telemetry);

        robot.turnLeftTillDegrees(345, 0.5, telemetry);

        robot.driveForwardRotation(0.25, 0.35);

        robot.turnRightTillDegrees(45, 0.50, telemetry);

        robot.driveForwardRotationTurn(2.3, 0.35, 0.30);

        robot.driveForwardRotation(0.85, .35);

        robot.turnRightTillDegrees(25, 0.50, telemetry);

        robot.dropMarker();

        robot.driveBackwardRotation(0.1, 0.35);

        robot.turnLeftTillDegrees(325, 0.50, telemetry);

        robot.driveBackwardRotationAlignWall(3.0, 0.4, 7, telemetry, autonomusTimer);

        if (autonomusTimer.seconds() <= 24)
            robot.driveBackwardTillTime( 5000, 0.20, telemetry);

    }

}
