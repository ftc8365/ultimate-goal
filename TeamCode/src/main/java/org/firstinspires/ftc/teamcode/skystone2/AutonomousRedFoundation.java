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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;



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
@Autonomous(name="Auto Red Foundation", group="Autonomous")
@Disabled
public class AutonomousRedFoundation extends LinearOpMode {

    //////////////////////////////////////////////////////////////////////
    // Declare OpMode members
    //////////////////////////////////////////////////////////////////////

    ElapsedTime timer = new ElapsedTime();

//    ElapsedTime autonomusTimer = new ElapsedTime();

    SkystoneRobot robot = new SkystoneRobot();

    SkystoneRobot.SkystonePosition skystonePosition = SkystoneRobot.SkystonePosition.SKYSTONE_POSITION_1;

    int secondStoneDistanceMoved = 0;

    @Override
    public void runOpMode() {

        robot.setAllianceMode(SkystoneRobot.AllianceMode.ALLIANCE_RED);
        robot.setOpMode(this);
        robot.initGyroSensor();
        robot.initDriveMotors();
        robot.initIntakeMotors();
        robot.initFoundationServos();
        robot.initIntakeServos();
        robot.initLiftServos();
        robot.initRangeSensors();
        robot.initColorSensors();
        robot.initTensorFlowObjectDetectionWebcam();
        robot.initCameraServo();
        robot.raiseFoundationServos();

        robot.initV4BLState(SkystoneRobot.V4BLState.V4BL_STATE_INTAKE);
        robot.raiseGrabber();

        while (!opModeIsActive() && !isStopRequested()) {

            skystonePosition = robot.scanSkystone(skystonePosition);
            telemetry.addData("Skystone", skystonePosition);
            telemetry.addData("", "------------------------------");
            telemetry.addData("distance", robot.distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("Gyro Pos", robot.getCurrentPositionInDegrees());
            telemetry.addData("MotorFR Pos", robot.motorFR.getCurrentPosition());
            telemetry.addData("MotorFL Pos", robot.motorFL.getCurrentPosition());
            telemetry.addData("rangeSensorFront", robot.rangeSensorFront.getDistance(DistanceUnit.INCH));
            telemetry.addData("rangeSensorBack", robot.rangeSensorBack.getDistance(DistanceUnit.INCH));

            telemetry.addData("", "------------------------------");
            telemetry.addData(">", "Press Play to start");
            telemetry.update();
        }

        ///////////////////////////////////////
        // Start of program
        ///////////////////////////////////////

        startAuto();

        ////////////////
        // STEP 1
        ////////////////
        grabSkystone();

        ////////////////
        // STEP 2
        ////////////////
        grabFoundation();

        ////////////////
        // STEP 3
        ////////////////
        turnFoundation();

        ////////////////
        // STEP 4
        ////////////////
        boolean stonePlaced = placeSkystone();

        ////////////////
        // STEP 5
        ////////////////
        if (stonePlaced && !robot.stoneDetected())
        {
            grab2ndSkystone();

            deliver2ndSkystone();
        }

        moveUnderAllianceBridge();

        robot.stopDriveMotors();
        robot.turnIntakeoff();

//        telemetry.addData("done", robot.autonomusTimer.milliseconds());

//        telemetry.update();

//        // REMOVE LATER
//        while (opModeIsActive() ) {
//
//            sleep(100);
//        }

    }

    void startAuto() {
        robot.resetAutonomousTimer();
        robot.shutdownTensorFlow();
        robot.setLatchPosition(SkystoneRobot.LatchPosition.LATCH_POSITION_1);
        robot.servoCamera.setPosition(0.3);
    }


    void grabSkystone() {

//        telemetry.addData("grabSkyStone Begin", robot.autonomusTimer.milliseconds());

        if (!opModeIsActive())
            return;

        robot.turnIntakeOn(SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_IN);

        robot.driveForwardTillRotation(1.50,0.10,0.50,0, true, false);

        switch (skystonePosition) {
            case SKYSTONE_POSITION_1:
                robot.driveForwardTillRotation(2.00, 0.30, 0.30, 0, false, true);

                timer.reset();

                while (timer.milliseconds() < 2000) {
                    if (robot.stoneDetected())  {
                        robot.grabStone();
                        break;
                    }
                }

                robot.turnIntakeoff();

                //////////////// INCREASE if Slideswipes skybridge

                robot.driveBackwardTillRotation(0.50, 0.40, 0.40, 0, true, true);

                //////// go into this program if cannot go back more
                robot.curveBackwardTillRotation(false, 1.0, 0.6, 270, false, false);

                break;

            case SKYSTONE_POSITION_2:
            case SKYSTONE_POSITION_3:
                robot.turnLeftTillDegrees(270, true, true);

                if (skystonePosition == SkystoneRobot.SkystonePosition.SKYSTONE_POSITION_2 )
                    robot.driveBackwardTillRotation(0.35, 0.20,0.50, 270, false, true);
//                else
//                    robot.driveForwardTillRotation(0.0, 0.20,0.50, 270, false, true);

                robot.driveRightTillRotation(0.55, 0.50,0.50, 270, false, true);

                robot.driveForwardTillRotation(0.50, 0.30,0.30, 270, false, true);

                timer.reset();

                while (timer.milliseconds() < 2000) {
                    if (robot.stoneDetected())  {
                        robot.grabStone();
                        break;
                    }
                }

                robot.turnIntakeoff();

                robot.driveLeftTillRotation(0.50, 0.50,0.50, 270, true, true);

                break;
        }

//        telemetry.addData("grabSkyStone End", robot.autonomusTimer.milliseconds());

    }

    void grabFoundation() {
//        telemetry.addData("grabFoundation Begin", robot.autonomusTimer.milliseconds());

        if (!opModeIsActive())
            return;

        double distanceToGo = 3.75;

        switch (skystonePosition) {
            case SKYSTONE_POSITION_1:
                distanceToGo = 1.7;
                break;

            case SKYSTONE_POSITION_2:
                distanceToGo += 0.35;
                break;

            case SKYSTONE_POSITION_3:
                distanceToGo += 0.70;
                break;
        }

        robot.setFoundationServos(SkystoneRobot.FoundationServoPosition.FOUNDATION_SERVO_MIDDLE);

        robot.driveBackwardTillRotation(distanceToGo, 0.70,0.70, 270, false, false);

        robot.driveBackwardTillRange(28,0.70,0.35, 270, true,true);

        robot.turnLeftTillDegrees(180, true, true);

        int startBackupPos = robot.motorFR.getCurrentPosition();
        robot.driveBackwardTillTime(750,0.25,180,true);
        int endBackupPos = robot.motorFR.getCurrentPosition();

        robot.lowerFoundationServos();
        sleep(350);

        int foundationDistanceMoved = startBackupPos - endBackupPos;

        //////////////// INCREASE IF DOESN"T TOUCH PERIMETER

        robot.driveForwardTillTicks(foundationDistanceMoved /*+ 235*/,0.1,0.5,180,false,true);

//        telemetry.addData("grabFoundation End", robot.autonomusTimer.milliseconds());
    }

    void turnFoundation() {

        telemetry.addData("turnFoundation Begin", robot.autonomusTimer.milliseconds());

        if (opModeIsActive()) {
            robot.turnRightTillDegrees(270, 0.8, true, true);

            if (opModeIsActive())
                robot.raiseFoundationServos();

            /////UNCOMMENT WHEN NEEDED when robot is not within second lane

//            robot.driveRightTillRotation(0.25, 0.5,0.5, 270, false, true);

            robot.driveBackwardTillTime(250, 0.30, 270,false);

        }

        telemetry.addData("turnFoundation End", robot.autonomusTimer.milliseconds());
    }

    boolean placeSkystone() {
        telemetry.addData("dropSkystone Begin", robot.autonomusTimer.milliseconds());
        boolean placed = robot.placeStone();
        telemetry.addData("dropSkystone End", robot.autonomusTimer.milliseconds());

        return placed;
    }

    void moveUnderAllianceBridge() {
        telemetry.addData("moveUnderAllianceBridge Begin", robot.autonomusTimer.milliseconds());

        if (opModeIsActive()) {
            robot.driveForwardTillRotation(3.0, 0.60, 0.60, 270, false, true);
        }
        telemetry.addData("moveUnderAllianceBridge End", robot.autonomusTimer.milliseconds());
    }


    void grab2ndSkystone() {

        double rangeToGo = 0;
        double rotationToGo = 4.5;

        switch (skystonePosition) {
            case SKYSTONE_POSITION_1:
                rangeToGo = 30.5;
                rotationToGo = 4.5;
                break;

            case SKYSTONE_POSITION_2:
                rangeToGo = 22.5;
                rotationToGo += 0.5;
                break;

            case SKYSTONE_POSITION_3:
                rangeToGo = 15.5;
                rotationToGo += 0.9;
                break;
        }

        int motorFRStartingPos = robot.motorFR.getCurrentPosition();
        int motorFLStartingPos = robot.motorFL.getCurrentPosition();

        robot.driveForwardTillRotation(rotationToGo, 0.80, 0.80, 270, true, true);

        if (robot.continueAutonomus())
            robot.initV4BLState(SkystoneRobot.V4BLState.V4BL_STATE_INTAKE);

        if (robot.continueAutonomus())
            robot.raiseGrabber();

        telemetry.addData( "range started", robot.autonomusTimer.milliseconds());

        robot.driveForwardTillRange(rangeToGo, 0.35, 0.35, 270, true, true);

        int motorFRMoved = robot.motorFR.getCurrentPosition() - motorFRStartingPos;
        int motorFLMoved = robot.motorFL.getCurrentPosition() - motorFLStartingPos;

        secondStoneDistanceMoved = ( motorFRMoved + motorFLMoved ) / 2 + (int)(0.35 * robot.TICK_PER_WHEEL_ROTATION);

        telemetry.addData( "range finished", robot.autonomusTimer.milliseconds());

        robot.turnIntakeOn(SkystoneRobot.IntakeDirection.INTAKE_DIRECTION_IN);

        robot.driveRightTillRotation(0.50, 0.50,0.50, 270, false, false);

        robot.driveForwardTillRotation(0.50, 0.35,0.35, 270, false, true);

        timer.reset();

        while (timer.milliseconds() < 2000) {
            if (robot.stoneDetected())  {
                robot.grabStone();
                break;
            }
        }

        robot.turnIntakeoff();

        robot.driveLeftTillRotation(0.50, 0.50,0.50, 270, false, true);
    }


    void deliver2ndSkystone() {

        telemetry.addData( "return started", robot.autonomusTimer.milliseconds());
        telemetry.addData( "secondStoneDistanceMoved", secondStoneDistanceMoved);

        robot.driveBackwardTillTicks(secondStoneDistanceMoved - 50, 0.80, 0.80, 270, true, true, 1000, 3500);

        telemetry.addData( "return finished", robot.autonomusTimer.milliseconds());

        telemetry.addData( "dropStone2 started", robot.autonomusTimer.milliseconds());

        if (robot.stoneDetected())
            robot.dropStone2();

        telemetry.addData( "dropStone2 ended", robot.autonomusTimer.milliseconds());

    }

}
