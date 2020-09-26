package org.firstinspires.ftc.teamcode.ultimategoal.robot;


import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.List;

public class Robot {

    ///////////////////////////////////
    // Declare constants
    ///////////////////////////////////

    final double ODOMETRY_WHEEL_RADIUS               = ( 38 * 0.039370 ) / 2;  // Nexus Omni wheel is 38mm in diagram, convert to inches
    final int    ODOMETRY_WHEEL_TICK_PER_ROTATION    = 1440;                   // Based on E8T spec
    final int    AUTONOMOUS_DURATION_MSEC            = 29800;

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare motors variables
    ////////////////////////////////////////////////////////////////////////////////////
    RevBulkData      bulkData;
    AnalogInput      a0, a1, a2, a3;
    DigitalChannel   d0, d1, d2, d3, d4, d5, d6, d7;
    ExpansionHubEx   expansionHub;

    ExpansionHubMotor motorFR       = null;
    ExpansionHubMotor motorFL       = null;
    ExpansionHubMotor motorBR       = null;
    ExpansionHubMotor motorBL       = null;

    PIDController     pidController = new PIDController();
    KalmanFilter      kalmanFilter  = new KalmanFilter();

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare tensorflow variables
    ////////////////////////////////////////////////////////////////////////////////////
    ComputerVision    computerVision;

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare sensors
    ////////////////////////////////////////////////////////////////////////////////////
    IntegratingGyroscope        gyro;
    NavxMicroNavigationSensor   navxMicro;

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare Servos
    ////////////////////////////////////////////////////////////////////////////////////
    Servo servoExample;

    //////////////////////////////////////////
    // Declare sensors
    //////////////////////////////////////////
    ElapsedTime autonomusTimer;
    final LinearOpMode opMode;
    boolean runningAutonomous;


    //----------------------------------------------------------------------------------------------------------------------------------
    // Class Methods Starts Here
    //----------------------------------------------------------------------------------------------------------------------------------

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Constructor
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Robot( LinearOpMode opMode ) {
        this.autonomusTimer = new ElapsedTime();
        this.opMode = opMode;
        this.runningAutonomous = false;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // resetAutonomousTimer
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void resetAutonomousTimer() {
        autonomusTimer.reset();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // initDriveMotors
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void initDriveTrain() {
        expansionHub = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        motorFR = (ExpansionHubMotor) opMode.hardwareMap.dcMotor.get("motorFR");
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL = (ExpansionHubMotor) opMode.hardwareMap.dcMotor.get("motorFL");
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBR = (ExpansionHubMotor) opMode.hardwareMap.dcMotor.get("motorBR");
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBL = (ExpansionHubMotor) opMode.hardwareMap.dcMotor.get("motorBL");
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        navxMicro = opMode.hardwareMap.get(NavxMicroNavigationSensor.class, "gyro_sensor");
        gyro = (IntegratingGyroscope) navxMicro;

        while (navxMicro.isCalibrating()) {
            opMode.sleep(50);
        }
    }

    public void getInputData() {
        bulkData = expansionHub.getBulkInputData();
    }

    public void initComputerVision() {
        this.computerVision = new ComputerVision(this.opMode);

        this.computerVision.initVuforia();
        this.computerVision.initTfod();
        this.computerVision.activate();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public ComputerVision getComputerVision() {
        return this.computerVision;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void shutdown() {
        this.stopDriveMotors();
        this.computerVision.shutdown();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public boolean inInitializationState() {
        return (!opMode.opModeIsActive() && !opMode.isStopRequested());
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void setRunningAutonomous(boolean autonomous) {
        this.runningAutonomous = autonomous;
    }

    public double getCurrentHeading() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double fromDegrees = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        return fromDegrees;
    }

    public double getCurrentPositionInDegrees() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double fromDegrees = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);

        // degrees
        // 0, -45, -90, -180
        // convert that to 0, 45, 90, 180

        double toDegrees;

        if (fromDegrees < 0)
            toDegrees = fromDegrees * -1;
        else
            toDegrees = 360 - fromDegrees;

        return toDegrees % 360;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // setDriveMotorPower
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void setDriveMotorPower(double powerFR, double powerFL, double powerBR, double powerBL) {
        this.motorFL.setPower(powerFL);
        this.motorFR.setPower(powerFR);
        this.motorBL.setPower(powerBL);
        this.motorBR.setPower(powerBR);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // stopAllMotors
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void stopDriveMotors() {
        this.motorFL.setPower(0);
        this.motorFR.setPower(0);
        this.motorBL.setPower(0);
        this.motorBR.setPower(0);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // continueAutonomus
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    boolean continueAutonomus() {
        if (!this.runningAutonomous)
            return true;
        else
            return (opMode.opModeIsActive()) && (autonomusTimer.milliseconds() < AUTONOMOUS_DURATION_MSEC);
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveForwardTillDistance
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForwardTillDistance(double distanceInches, double initPower, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors) {

        double inchesPerRotation = 2 * Math.PI * ODOMETRY_WHEEL_RADIUS;

        double ticks = (distanceInches / inchesPerRotation) * ODOMETRY_WHEEL_TICK_PER_ROTATION;

        driveForwardTillTicks((int) ticks, initPower, targetPower, targetHeading, rampDown, stopMotors);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveForwardTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForwardTillTicks(int ticks, double initPower, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors) {
        boolean useGyroToAlign = (this.gyro != null && targetHeading >= 0) ? true : false;
        int initPosition = getLeftOdometryPostion();
        int ticksToGo = 0;
        double power = initPower;

        while (continueAutonomus()) {
            ticksToGo = ticks - (getLeftOdometryPostion() - initPosition);

            if (ticksToGo <= 0)
                break;

            //get power based on distance
            power = pidController.getDrivePower(power, ticksToGo, targetPower, rampDown);

            double powerRight = power;
            double powerLeft = power;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            if (useGyroToAlign) {
                double currentPosition = this.getCurrentPositionInDegrees();
                double headingChange = currentPosition - targetHeading;

                if (headingChange > 180 && targetHeading <= 30) {
                    headingChange -= 360;
                }

                powerRight += 2 * (headingChange / 100);
                powerLeft -= 2 * (headingChange / 100);
            }

            motorFR.setPower(powerRight);
            motorFL.setPower(powerLeft);
            motorBR.setPower(powerRight);
            motorBL.setPower(powerLeft);

        }

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getLeftOdometryPostion() {
        return bulkData.getMotorCurrentPosition(2);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getRightOdometryPostion() {
        return bulkData.getMotorCurrentPosition(0);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getCenterOdometryPosition() {
        return bulkData.getMotorCurrentPosition(1);
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveFBackwardTillDistance
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackwardTillDistance( double distanceInches, double initPower, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {

        double inchesPerRotation = 2 * Math.PI * ODOMETRY_WHEEL_RADIUS;

        double ticks = (distanceInches / inchesPerRotation) * ODOMETRY_WHEEL_TICK_PER_ROTATION;

        this.opMode.telemetry.addData("inchesPerRotation", inchesPerRotation);
        this.opMode.telemetry.addData( "ticks", ticks );

        driveBackwardTillTicks( (int)ticks, initPower, targetPower, targetHeading, rampDown, stopMotors, -1, -1);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackwardTillTicks( int ticks, double initPower, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors, int rampDownRange, int maxTime )
    {
        boolean useGyroToAlign  = (this.gyro != null && targetHeading >= 0) ? true : false;
        int initPosition        = getLeftOdometryPostion();
        int ticksToGo           = 0;
        double power            = initPower;
        double startingTime     = 0;

        if (maxTime > 0)
            startingTime = autonomusTimer.milliseconds();

        while (continueAutonomus()) {
            ticksToGo = ticks - (initPosition - getLeftOdometryPostion());
            if (ticksToGo <= 0)
                break;

            this.opMode.telemetry.addData( "ticks", ticks );
            this.opMode.telemetry.addData( "init", initPosition );
            this.opMode.telemetry.addData( "curr", getLeftOdometryPostion() );
            this.opMode.telemetry.addData( "diff", ticksToGo );
            this.opMode.telemetry.update();

            if (maxTime > 0) {
                if ((autonomusTimer.milliseconds() - startingTime) > maxTime)
                    break;
            }

            power = pidController.getDrivePower(power, ticksToGo, targetPower, rampDown);

            double powerRight = power;
            double powerLeft  = power;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            if (useGyroToAlign) {
                double currentPosition = this.getCurrentPositionInDegrees();
                double headingChange   = currentPosition - targetHeading;

                if (headingChange > 180 && targetHeading == 0) {
                    headingChange -= 360;
                }
                powerRight -=  2 * (headingChange / 100);
                powerLeft  +=  2 * (headingChange / 100);

            }

            motorFR.setPower( powerRight * -1);
            motorFL.setPower( powerLeft * -1);
            motorBR.setPower( powerRight * -1);
            motorBL.setPower( powerLeft * -1);
        }

        if (stopMotors) {
            stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveBackwardTillTime
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackwardTillTime( long timeInMilliSeconds, double targetPower, int targetHeading, boolean stopMotors ) {

        boolean useGyroToAlign  = (this.gyro != null && targetHeading >= 0) ? true : false;
        double power            = targetPower;
        double currentTime      = autonomusTimer.milliseconds();

        while (continueAutonomus()) {

            if (autonomusTimer.milliseconds() - currentTime >= timeInMilliSeconds)
                break;

            power = targetPower;

            double powerRight = power;
            double powerLeft  = power;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            if (useGyroToAlign) {
                double currentPosition = this.getCurrentPositionInDegrees();
                double headingChange   = currentPosition - targetHeading;

                if (headingChange > 180 && targetHeading == 0) {
                    headingChange -= 360;
                }
                powerRight -=  2 * (headingChange / 100);
                powerLeft  +=  2 * (headingChange / 100);

            }

            motorFR.setPower( powerRight * -1);
            motorFL.setPower( powerLeft * -1);
            motorBR.setPower( powerRight * -1);
            motorBL.setPower( powerLeft * -1);
        }

        if (stopMotors) {
            stopDriveMotors();
        }
    }



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveLeftTillDistance
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveLeftTillDistance( double distanceInInches, double initPower, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {

        double inchesPerRotation = 2 * Math.PI * ODOMETRY_WHEEL_RADIUS;

        double ticks = (distanceInInches / inchesPerRotation) * ODOMETRY_WHEEL_TICK_PER_ROTATION;

        driveLeftTillTicks( (int)ticks, initPower, targetPower, targetHeading, rampDown, stopMotors );
    }

    public void driveLeftTillTicks( int ticks, double initPower, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors )
    {
        boolean useGyroToAlign  = (this.gyro != null && targetHeading >= 0) ? true : false;
        int initPosition        = getCenterOdometryPosition();
        int ticksToGo           = 0;
        double power            = initPower;

        while (continueAutonomus()) {

            ticksToGo = ticks - (getCenterOdometryPosition() - initPosition);

            if (ticksToGo <= 0)
                break;

            power = pidController.getSidewayDrivePower(power, ticksToGo, targetPower, rampDown);

            double powerFront = power;
            double powerBack  = power;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            if (useGyroToAlign) {
                double currentPosition = this.getCurrentPositionInDegrees();
                double headingChange   = currentPosition - targetHeading;

                if (headingChange > 180 && targetHeading == 0) {
                    headingChange -= 360;
                }
                powerFront +=  2 * (headingChange / 100);
                powerBack  -=  2 * (headingChange / 100);
            }

            motorFR.setPower( powerFront );
            motorFL.setPower( powerFront * -1 );
            motorBR.setPower( powerBack * -1 );
            motorBL.setPower( powerBack );

        }

        if (stopMotors) {
            stopDriveMotors();
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveRightTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveRightTillDistance( double distanceInInches, double initPower, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {

        double inchesPerRotation = 2 * Math.PI * ODOMETRY_WHEEL_RADIUS;

        double ticks = (distanceInInches / inchesPerRotation) * ODOMETRY_WHEEL_TICK_PER_ROTATION;

        driveRightTillTicks( (int)ticks, initPower, targetPower, targetHeading, rampDown, stopMotors );
    }

    public void driveRightTillTicks( int ticks, double initPower, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors )
    {
        boolean useGyroToAlign  = (this.gyro != null && targetHeading >= 0) ? true : false;
        int initPosition        = getCenterOdometryPosition();
        int ticksToGo           = 0;
        double power            = initPower;

        while (continueAutonomus()) {

            ticksToGo = ticks - (initPosition - getCenterOdometryPosition());
            if (ticksToGo <= 0)
                break;

            power = pidController.getSidewayDrivePower(power, ticksToGo, targetPower, rampDown);

            double powerFront = power;
            double powerBack  = power;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            if (useGyroToAlign) {
                double currentPosition = this.getCurrentPositionInDegrees();
                double headingChange   = currentPosition - targetHeading;

                if (headingChange > 180 && targetHeading == 0) {
                    headingChange -= 360;
                }
                powerFront -=  2 * (headingChange / 100);
                powerBack  +=  2 * (headingChange / 100);
            }

            motorFR.setPower( powerFront * -1 );
            motorFL.setPower( powerFront );
            motorBR.setPower( powerBack );
            motorBL.setPower( powerBack * -1 );
        }

        if (stopMotors) {
            stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // turnRightTillDegrees
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void turnRightTillDegrees( int targetDegrees, double targetPower, boolean rampDown, boolean stopMotors )
    {
        double power = 0;
        double currentHeading = 0;
        double degressToGo = 0;
        double TURN_TOLERANCE = 5.0;

        while (continueAutonomus()) {

            currentHeading = getCurrentPositionInDegrees();
            if (currentHeading > 180 && targetDegrees < 180)
                currentHeading -= 360;

            degressToGo = targetDegrees - currentHeading;
            if (degressToGo <= TURN_TOLERANCE)
                break;

            power = pidController.getTurnPower(power, degressToGo, targetPower, rampDown);

            motorFR.setPower( -1 * power );
            motorFL.setPower(      power );
            motorBR.setPower( -1 * power );
            motorBL.setPower(      power );
        }

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // turnLeftTillDegrees
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void turnLeftTillDegrees( int targetDegrees, double targetPower, boolean rampDown, boolean stopMotors )
    {
        double power = 0.30;
        double currentHeading = 0;
        double degressToGo = 0;

        while (continueAutonomus()) {

            currentHeading = getCurrentPositionInDegrees();

            degressToGo = currentHeading - targetDegrees;

            if (degressToGo < -120)
                degressToGo += 360;

            if (degressToGo <= 0.5)
                break;

            power = pidController.getTurnPower(power, degressToGo, targetPower, rampDown);

            motorFR.setPower(      power );
            motorFL.setPower( -1 * power );
            motorBR.setPower(      power );
            motorBL.setPower( -1 * power );
        }

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }

    public TrajectoryBuilder trajectoryBuilder() {
        return new TrajectoryBuilder();
    }

    public void followTrajectory( Trajectory trajectory ) {
        List< Movement > movements = trajectory.getMovements();

        for (Movement movement : movements) {
            switch (movement.getType()) {
                case MOVE_FORWARD:
                    this.driveForwardTillDistance( movement.getDistance(),
                                                    movement.getConstraint().getInitPower(),
                                                    movement.getConstraint().getTargetPower(),
                                                    movement.getTargetHeading(),
                                                    movement.getConstraint().getRampDown(),
                                                    movement.getConstraint().getStopMotor());
                    break;
                case MOVE_BACKWARD:
                    this.driveBackwardTillDistance( movement.getDistance(),
                                                    movement.getConstraint().getInitPower(),
                                                    movement.getConstraint().getTargetPower(),
                                                    movement.getTargetHeading(),
                                                    movement.getConstraint().getRampDown(),
                                                    movement.getConstraint().getStopMotor());
                    break;
                case STRAFE_LEFT:
                    this.driveLeftTillDistance( movement.getDistance(),
                                                movement.getConstraint().getInitPower(),
                                                movement.getConstraint().getTargetPower(),
                                                movement.getTargetHeading(),
                                                movement.getConstraint().getRampDown(),
                                                movement.getConstraint().getStopMotor());
                    break;
                case STRAFE_RIGHT:
                    this.driveRightTillDistance( movement.getDistance(),
                                                    movement.getConstraint().getInitPower(),
                                                    movement.getConstraint().getTargetPower(),
                                                    movement.getTargetHeading(),
                                                    movement.getConstraint().getRampDown(),
                                                    movement.getConstraint().getStopMotor());
                    break;
                case TURN_LEFT:
                    this.turnLeftTillDegrees( movement.getTargetHeading(),
                                                movement.getConstraint().getTargetPower(),
                                                movement.getConstraint().getRampDown(),
                                                movement.getConstraint().getStopMotor() );
                    break;
                case TURN_RIGHT:
                    this.turnRightTillDegrees( movement.getTargetHeading(),
                                                movement.getConstraint().getTargetPower(),
                                                movement.getConstraint().getRampDown(),
                                                movement.getConstraint().getStopMotor() );
                    break;
                case STOP:
                    this.stopDriveMotors();
                    break;
            }
        }
    }
}
