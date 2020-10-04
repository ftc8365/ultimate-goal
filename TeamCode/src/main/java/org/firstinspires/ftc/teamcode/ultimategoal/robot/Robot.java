package org.firstinspires.ftc.teamcode.ultimategoal.robot;


import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.List;

public class Robot {

    //////////////////////////////////////////////////////////////////////
    // Declare constants
    //////////////////////////////////////////////////////////////////////

    public final double ODOMETRY_WHEEL_RADIUS              = 1.49606 / 2;  // Nexus Omni wheel is 38mm in diagram, convert to inches
    public final int    ODOMETRY_WHEEL_TICKS_PER_ROTATION  = 1440;                   // Based on E8T spec
    public final int    ODOMETRY_WHEEL_TICKS_PER_INCH      = (int)((double)ODOMETRY_WHEEL_TICKS_PER_ROTATION / (Math.PI * ODOMETRY_WHEEL_RADIUS * 2));
    public final double ODEMOTRY_WHEEL_DIAMETER            = 14.0;

    final int    AUTONOMOUS_DURATION_MSEC           = 29800;

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare motors variables
    ////////////////////////////////////////////////////////////////////////////////////
    DcMotorEx       motorFR;
    DcMotorEx       motorFL;
    DcMotorEx       motorBR;
    DcMotorEx       motorBL;

    LynxModule      controlHub;

//    RevBulkData         bulkData;
//    ExpansionHubEx      expansionHub;

//    ExpansionHubMotor   motorFR = null;
//    ExpansionHubMotor   motorFL = null;
//    ExpansionHubMotor   motorBR = null;
//    ExpansionHubMotor   motorBL = null;

    double              curDrivePower  = 0;

    PIDController   pidController = new PIDController(ODOMETRY_WHEEL_TICKS_PER_ROTATION);
    KalmanFilter    kalmanFilter  = new KalmanFilter();


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
    ElapsedTime         autonomusTimer;
    final LinearOpMode  opMode;
    boolean             runningAutonomous;

    //----------------------------------------------------------------------------------------------------------------------------------
    // Class Methods Starts Here
    //----------------------------------------------------------------------------------------------------------------------------------

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Constructor
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Robot( LinearOpMode opMode ) {
        this.autonomusTimer     = new ElapsedTime();
        this.opMode             = opMode;
        this.runningAutonomous  = false;
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
//        expansionHub = opMode.hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        motorFR = opMode.hardwareMap.get(DcMotorEx.class, "motorFR");  // Configure the robot to use these 4 motor names,
        //motorFR = (ExpansionHubMotor) opMode.hardwareMap.dcMotor.get("motorFR");
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorFL = opMode.hardwareMap.get(DcMotorEx.class, "motorFL");  // or change these strings to match your existing Robot Configuration.
        //motorFL = (ExpansionHubMotor) opMode.hardwareMap.dcMotor.get("motorFL");
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorBR = opMode.hardwareMap.get(DcMotorEx.class, "motorBR");
        //motorBR = (ExpansionHubMotor) opMode.hardwareMap.dcMotor.get("motorBR");
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorBL = opMode.hardwareMap.get(DcMotorEx.class, "motorBL");
        //motorBL = (ExpansionHubMotor) opMode.hardwareMap.dcMotor.get("motorBL");
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs = opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            this.controlHub = module;
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        navxMicro = opMode.hardwareMap.get(NavxMicroNavigationSensor.class, "gyro_sensor");
        gyro = (IntegratingGyroscope) navxMicro;

        while (navxMicro.isCalibrating()) {
            opMode.sleep(50);
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void clearBulkCache() {
        this.controlHub.clearBulkCache();
        //bulkData = expansionHub.getBulkInputData();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
    public void setRunningAutonomous(boolean autonomous) {
        this.runningAutonomous = autonomous;
    }

    public double getCurrentHeading() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
}

    public int getInitLeftOdometryPosition() {
        return 0;
    }

    public int getInitRightOdometryPosition() {
        return 0;
    }

    // Using only motor encoders (via odometry), we can get rate of 300 cycles per second
    // adding gyro sensor, the rate reduce to 80 per second
    //
    // Excercise - figure out how to calculate robot heading based on
    public double getCurrentPositionInDegrees() {
        int left    = this.getLeftOdometryPosition() - getInitLeftOdometryPosition();
        int right   = this.getRightOdometryPosition() - getInitRightOdometryPosition();
        int diff    = right - left;

        double heading =  -180 * diff / ( ODEMOTRY_WHEEL_DIAMETER * Math.PI * ODOMETRY_WHEEL_TICKS_PER_INCH ) ;

        if (heading < 0)
            heading = 360 + heading;

        return heading;
    }

    public double getCurrentPositionInDegreesX() {
        double fromDegrees = getCurrentHeading();

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
        this.motorFR.setPower(powerFR);
        this.motorFL.setPower(powerFL);
        this.motorBR.setPower(powerBR);
        this.motorBL.setPower(powerBL);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // stopAllMotors
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void stopDriveMotors() {
        this.curDrivePower = 0;
        this.motorFR.setPower(0);
        this.motorFL.setPower(0);
        this.motorBR.setPower(0);
        this.motorBL.setPower(0);
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
    public int getRightOdometryPosition() {
        //return bulkData.getMotorCurrentPosition(2);
        return motorBR.getCurrentPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getLeftOdometryPosition() {
        //return bulkData.getMotorCurrentPosition(0);
        return motorFR.getCurrentPosition() * -1;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getCenterOdometryPosition() {
        //return bulkData.getMotorCurrentPosition(3);
        return motorBL.getCurrentPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveForwardTillDistance
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForwardTillDistance(double distanceInches, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors) {
        driveForwardTillTicks((int) distanceInches * ODOMETRY_WHEEL_TICKS_PER_INCH, targetPower, targetHeading, rampDown, stopMotors);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveForwardTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForwardTillTicks(int targetTicks, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors) {
        clearBulkCache();
//        boolean useGyroToAlign = (this.gyro != null && targetHeading >= 0) ? true : false;
        int initPosition = getRightOdometryPosition();

        while (continueAutonomus()) {
            int ticksToGo = targetTicks - (getRightOdometryPosition() - initPosition);
            if (ticksToGo <= 0)
                break;

            //get power based on distance
            this.curDrivePower = pidController.getDrivePower(this.curDrivePower, ticksToGo, targetPower, rampDown);

            double powerRight = this.curDrivePower;
            double powerLeft = this.curDrivePower;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            double currentPosition = this.getCurrentPositionInDegrees();
            double headingChange = currentPosition - targetHeading;

            if (headingChange > 180 && targetHeading <= 30) {
                headingChange -= 360;
            }

            powerRight += 2 * (headingChange / 100);
            powerLeft -= 2 * (headingChange / 100);

            motorFR.setPower(powerRight);
            motorFL.setPower(powerLeft);
            motorBR.setPower(powerRight);
            motorBL.setPower(powerLeft);

            clearBulkCache();
        }

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveFBackwardTillDistance
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackwardTillDistance( double distanceInches, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {
        driveBackwardTillTicks( (int) distanceInches * ODOMETRY_WHEEL_TICKS_PER_INCH, targetPower, targetHeading, rampDown, stopMotors, -1, -1);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackwardTillTicks( int ticks, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors, int rampDownRange, int maxTime ) {
        this.clearBulkCache();

        int initPosition        = getRightOdometryPosition();
        int ticksToGo           = 0;
        double startingTime     = 0;

        if (maxTime > 0)
            startingTime = autonomusTimer.milliseconds();

        while (continueAutonomus()) {
            ticksToGo = ticks - (initPosition - getRightOdometryPosition());
            if (ticksToGo <= 0)
                break;

            if (maxTime > 0) {
                if ((autonomusTimer.milliseconds() - startingTime) > maxTime)
                    break;
            }

            this.curDrivePower = pidController.getDrivePower(this.curDrivePower, ticksToGo, targetPower, rampDown);

            double powerRight = this.curDrivePower;
            double powerLeft  = this.curDrivePower;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            double currentPosition = this.getCurrentPositionInDegrees();
            double headingChange   = currentPosition - targetHeading;

            if (headingChange > 180 && targetHeading == 0) {
                headingChange -= 360;
            }
            powerRight -=  2 * (headingChange / 100);
            powerLeft  +=  2 * (headingChange / 100);

            motorFR.setPower( powerRight * -1);
            motorFL.setPower( powerLeft * -1);
            motorBR.setPower( powerRight * -1);
            motorBL.setPower( powerLeft * -1);

            clearBulkCache();
        }

        if (stopMotors) {
            stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveLeftTillDistance
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveLeftTillDistance( double distanceInInches, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {
        driveLeftTillTicks( (int) distanceInInches * ODOMETRY_WHEEL_TICKS_PER_INCH, targetPower, targetHeading, rampDown, stopMotors );
    }

    public void driveLeftTillTicks( int ticks, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {
        this.clearBulkCache();

//        boolean useGyroToAlign  = (this.gyro != null && targetHeading >= 0) ? true : false;
        int initPosition        = getCenterOdometryPosition();
        int ticksToGo           = 0;

        while (continueAutonomus()) {
            ticksToGo = ticks - (getCenterOdometryPosition() - initPosition);
            if (ticksToGo <= 0)
                break;

            this.curDrivePower = pidController.getSidewayDrivePower(this.curDrivePower, ticksToGo, targetPower, rampDown);

            double powerFront = this.curDrivePower;
            double powerBack  = this.curDrivePower;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            double currentPosition = this.getCurrentPositionInDegrees();
            double headingChange   = currentPosition - targetHeading;

            if (headingChange > 180 && targetHeading == 0) {
                headingChange -= 360;
            }
            powerFront +=  2 * (headingChange / 100);
            powerBack  -=  2 * (headingChange / 100);

            motorFR.setPower( powerFront );
            motorFL.setPower( powerFront * -1 );
            motorBR.setPower( powerBack * -1 );
            motorBL.setPower( powerBack );

            clearBulkCache();
        }

        if (stopMotors) {
            stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveRightTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveRightTillDistance( double distanceInInches, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {
        driveRightTillTicks( (int) distanceInInches * ODOMETRY_WHEEL_TICKS_PER_INCH, targetPower, targetHeading, rampDown, stopMotors );
    }

    public void driveRightTillTicks( int ticks, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {
        this.clearBulkCache();

        int initPosition        = getCenterOdometryPosition();
        int ticksToGo           = 0;

        while (continueAutonomus()) {
            ticksToGo = ticks - (initPosition - getCenterOdometryPosition());
            if (ticksToGo <= 0)
                break;

            this.curDrivePower = pidController.getSidewayDrivePower(this.curDrivePower, ticksToGo, targetPower, rampDown);

            double powerFront = this.curDrivePower;
            double powerBack  = this.curDrivePower;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            double currentPosition = this.getCurrentPositionInDegrees();
            double headingChange   = currentPosition - targetHeading;

            if (headingChange > 180 && targetHeading == 0) {
                headingChange -= 360;
            }
            powerFront -=  2 * (headingChange / 100);
            powerBack  +=  2 * (headingChange / 100);

            motorFR.setPower( powerFront * -1 );
            motorFL.setPower( powerFront );
            motorBR.setPower( powerBack );
            motorBL.setPower( powerBack * -1 );

            clearBulkCache();
        }

        if (stopMotors) {
            stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // turnRightTillDegrees
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void turnRightTillDegrees( int targetDegrees, double targetPower, boolean rampDown, boolean stopMotors ) {
        double currentHeading   = 0;
        double degressToGo      = 0;
        double TURN_TOLERANCE   = 5.0;

        while (continueAutonomus()) {
            this.clearBulkCache();
            currentHeading = getCurrentPositionInDegrees();
            if (currentHeading > 180 && targetDegrees < 180)
                currentHeading -= 360;

            degressToGo = targetDegrees - currentHeading;
            if (degressToGo <= TURN_TOLERANCE)
                break;

            this.curDrivePower = pidController.getTurnPower(this.curDrivePower, degressToGo, targetPower, rampDown);

            motorFR.setPower( -1 * this.curDrivePower );
            motorFL.setPower(      this.curDrivePower );
            motorBR.setPower( -1 * this.curDrivePower );
            motorBL.setPower(      this.curDrivePower );
        }

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // turnLeftTillDegrees
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void turnLeftTillDegrees( int targetDegrees, double targetPower, boolean rampDown, boolean stopMotors ) {
        double currentHeading = 0;
        double degressToGo = 0;

        while (continueAutonomus()) {
            this.clearBulkCache();
            currentHeading = getCurrentPositionInDegrees();

            degressToGo = currentHeading - targetDegrees;

            if (degressToGo < -120)
                degressToGo += 360;

            if (degressToGo <= 0.5)
                break;

            this.curDrivePower = pidController.getTurnPower(this.curDrivePower, degressToGo, targetPower, rampDown);

            motorFR.setPower(      this.curDrivePower );
            motorFL.setPower( -1 * this.curDrivePower );
            motorBR.setPower(      this.curDrivePower );
            motorBL.setPower( -1 * this.curDrivePower );
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
                                                    movement.getConstraint().getTargetPower(),
                                                    movement.getTargetHeading(),
                                                    movement.getConstraint().getRampDown(),
                                                    movement.getConstraint().getStopMotor());
                    break;
                case MOVE_BACKWARD:
                    this.driveBackwardTillDistance( movement.getDistance(),
                                                    movement.getConstraint().getTargetPower(),
                                                    movement.getTargetHeading(),
                                                    movement.getConstraint().getRampDown(),
                                                    movement.getConstraint().getStopMotor());
                    break;
                case STRAFE_LEFT:
                    this.driveLeftTillDistance( movement.getDistance(),
                                                movement.getConstraint().getTargetPower(),
                                                movement.getTargetHeading(),
                                                movement.getConstraint().getRampDown(),
                                                movement.getConstraint().getStopMotor());
                    break;
                case STRAFE_RIGHT:
                    this.driveRightTillDistance( movement.getDistance(),
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
