package org.firstinspires.ftc.teamcode.ultimategoal.robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.KalmanFilter;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.PIDController;

import java.util.List;

public class MecanumDriveTrain {

    //////////////////////////////////////////////////////////////////////
    // Declare constants
    //////////////////////////////////////////////////////////////////////

    public final double ODOMETRY_WHEEL_RADIUS               = 1.49606 / 2;       // Nexus Omni wheel is 38mm in diagram, convert to inches
    public final int    ODOMETRY_WHEEL_TICKS_PER_ROTATION   = 1440;              // Based on E8T encoder spec
    public final int    ODOMETRY_WHEEL_TICKS_PER_INCH       = (int)((double)ODOMETRY_WHEEL_TICKS_PER_ROTATION / (Math.PI * ODOMETRY_WHEEL_RADIUS * 2));
    public final double ODOMETRY_WHEEL_DIAMETER             = 13.40;             // Width of left & right odometer wheels
    public final double TURN_TOLERANCE_IN_DEGREES           = 1.5;

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare motors variables
    ////////////////////////////////////////////////////////////////////////////////////
    DcMotorEx       motorFR;
    DcMotorEx       motorFL;
    DcMotorEx       motorBR;
    DcMotorEx       motorBL;
    LynxModule      controlHub;

    double          curDrivePower  = 0;
    int             initRightOdometryPosition = 0;
    int             initLeftOdometryPosition = 0;
    int             initCenterOdometryPosition = 0;

    PIDController   pidController;
    KalmanFilter    kalmanFilter;

    //////////////////////////////////////////
    // Declare reference to main robot
    //////////////////////////////////////////
    Robot               robot;

    //----------------------------------------------------------------------------------------------------------------------------------
    // Class Methods Starts Here
    //----------------------------------------------------------------------------------------------------------------------------------

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Constructor
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public MecanumDriveTrain(Robot robot ) {
        this.robot          = robot;
        this.pidController  = new PIDController(ODOMETRY_WHEEL_TICKS_PER_ROTATION);
        this.kalmanFilter   = new KalmanFilter();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // initDriveMotors
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void init() {
        motorFR = robot.opMode.hardwareMap.get(DcMotorEx.class, "motorFR");  // Configure the robot to use these 4 motor names,
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL = robot.opMode.hardwareMap.get(DcMotorEx.class, "motorFL");  // or change these strings to match your existing Robot Configuration.
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBR = robot.opMode.hardwareMap.get(DcMotorEx.class, "motorBR");
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBL = robot.opMode.hardwareMap.get(DcMotorEx.class, "motorBL");
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.
        List<LynxModule> allHubs = robot.opMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            this.controlHub = module;
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        clearBulkCache();
        this.initLeftOdometryPosition   = this.getLeftOdometryPosition();
        this.initRightOdometryPosition  = this.getRightOdometryPosition();
        this.initCenterOdometryPosition = this.getCenterOdometryPosition();

    }

    public void setDriveTrainZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motorFR.setZeroPowerBehavior(behavior);
        motorFL.setZeroPowerBehavior(behavior);
        motorBR.setZeroPowerBehavior(behavior);
        motorBL.setZeroPowerBehavior(behavior);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void clearBulkCache() {
        //this.controlHub.clearBulkCache();
    }

    public int getInitLeftOdometryPosition() {
        return this.initLeftOdometryPosition;
    }

    public int getInitRightOdometryPosition() {
        return this.initRightOdometryPosition;
    }

//    public double getCurrentPositionInDegrees() {
//        return getCurrentPositionInDegreesUsingOdometry();
//    }

    // Using only motor encoders (via odometry), we can get rate of 300 cycles per second
    // adding gyro sensor, the rate reduce to 80 per second
    //
    public double getCurrentPositionInDegreesUsingOdometry() {
        return getCurrentPositionInDegreesUsingOdometry(this.getRightOdometryPosition(), this.getLeftOdometryPosition());
    }

    public double getCurrentPositionInDegreesUsingOdometry(int rightPosition, int leftPosition) {
        int left    = leftPosition - getInitLeftOdometryPosition();
        int right   = rightPosition - getInitRightOdometryPosition();
        int diff    = left - right;

        double heading = 180 * diff/(ODOMETRY_WHEEL_TICKS_PER_INCH * (ODOMETRY_WHEEL_DIAMETER * Math.PI));

        if (heading < 0){
            heading = 360 + heading;
        }
        if (heading > 360){
            heading = heading % 360;
        }
        return heading;
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
    // stop All Motors
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void stop() {
        this.curDrivePower = 0;
        this.motorFR.setPower(0);
        this.motorFL.setPower(0);
        this.motorBR.setPower(0);
        this.motorBL.setPower(0);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getRightOdometryPosition() {
        return motorFL.getCurrentPosition() * -1;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getLeftOdometryPosition() {
        return motorBL.getCurrentPosition() ;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getCenterOdometryPosition() {
        return motorFL.getCurrentPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveForwardTillDistance
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForwardTillDistance(double distanceInInches, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors) {
        driveForwardTillTicks((int) distanceInInches * ODOMETRY_WHEEL_TICKS_PER_INCH, targetPower, targetHeading, rampDown, stopMotors);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveForwardTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForwardTillTicks(int targetTicks, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors) {

        clearBulkCache();

        int initRightPosition = getRightOdometryPosition();
        int initLeftPosition  = getLeftOdometryPosition();

        while (robot.continueMotion()) {
            int rightPosition  = this.getRightOdometryPosition();
            int leftPosition = this.getLeftOdometryPosition();

            int ticksMoved = ((rightPosition - initRightPosition) + (leftPosition - initLeftPosition) ) / 2 ;

            int ticksToGo = targetTicks - ticksMoved;
            if (ticksToGo <= 0)
                break;

            //get power based on distance
            this.curDrivePower = pidController.getDrivePower(this.curDrivePower, ticksToGo, targetPower, rampDown);

            double powerRight = this.curDrivePower;
            double powerLeft = this.curDrivePower;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            double currentPosition = this.getCurrentPositionInDegreesUsingOdometry(rightPosition, leftPosition);
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
            stop();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveFBackwardTillDistance
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackwardTillDistance( double distanceInInches, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {
        driveBackwardTillTicks( (int) distanceInInches * ODOMETRY_WHEEL_TICKS_PER_INCH, targetPower, targetHeading, rampDown, stopMotors );
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackwardTillTicks( int targetTicks, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {
        this.clearBulkCache();

        int initRightPosition   = getRightOdometryPosition();
        int initLeftPosition    = getLeftOdometryPosition();
        int rightPosition       = initRightPosition;
        int leftPosition        = initLeftPosition;
        int ticksToGo           = 0;
        int counter             = 0;

        while (robot.continueMotion()) {
            if (counter > 0) {
                rightPosition = this.getRightOdometryPosition();
                leftPosition = this.getLeftOdometryPosition();
            }

            // Take the average of the left & right odometry position to determine ticksMoved
            int ticksMoved = ((initRightPosition - rightPosition) + (initLeftPosition - leftPosition) ) / 2 ;

            ticksToGo = targetTicks - (ticksMoved);
            if (ticksToGo <= 0)
                break;

            this.curDrivePower = pidController.getDrivePower(this.curDrivePower, ticksToGo, targetPower, rampDown);

            double powerRight = this.curDrivePower;
            double powerLeft  = this.curDrivePower;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            double currentPosition = this.getCurrentPositionInDegreesUsingOdometry(rightPosition, leftPosition);
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
            counter ++;
        }

        if (stopMotors) {
            stop();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveLeftTillDistance
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void strafeLeftTillDistance( double distanceInInches, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {
        strafeLeftTillTicks( (int) distanceInInches * ODOMETRY_WHEEL_TICKS_PER_INCH, targetPower, targetHeading, rampDown, stopMotors );
    }

    public void strafeLeftTillTicks( int ticks, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {
        this.clearBulkCache();

        int initPosition        = getCenterOdometryPosition();
        int ticksToGo           = 0;

        while (robot.continueMotion()) {
            ticksToGo = ticks - (initPosition - getCenterOdometryPosition());
            if (ticksToGo <= 0)
                break;

            this.curDrivePower = pidController.getStrafeDrivePower(this.curDrivePower, ticksToGo, targetPower, rampDown);

            double powerFront = this.curDrivePower;
            double powerBack  = this.curDrivePower;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            double currentPosition = this.getCurrentPositionInDegreesUsingOdometry();
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
            stop();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveRightTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void strafeRightTillDistance( double distanceInInches, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {
        strafeRightTillTicks( (int) distanceInInches * ODOMETRY_WHEEL_TICKS_PER_INCH, targetPower, targetHeading, rampDown, stopMotors );
    }

    public void strafeRightTillTicks( int ticks, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {
        this.clearBulkCache();

        int initPosition        = getCenterOdometryPosition();
        int ticksToGo           = 0;

        while (robot.continueMotion()) {
            ticksToGo = ticks - (getCenterOdometryPosition() - initPosition);
            if (ticksToGo <= 0)
                break;

            this.curDrivePower = pidController.getStrafeDrivePower(this.curDrivePower, ticksToGo, targetPower, rampDown);

            double powerFront = this.curDrivePower;
            double powerBack  = this.curDrivePower;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            double currentPosition = this.getCurrentPositionInDegreesUsingOdometry();
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
            stop();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // turnRightTillDegrees
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void turnRightTillDegrees( int targetDegrees, double targetPower, boolean rampDown, boolean stopMotors ) {
        double currentHeading   = 0;
        double degressToGo      = 0;

        while (robot.continueMotion()) {
            this.clearBulkCache();

            currentHeading = getCurrentPositionInDegreesUsingOdometry();
            if (currentHeading > 180 && targetDegrees < 180)
                currentHeading -= 360;

            degressToGo = targetDegrees - currentHeading;
            if (degressToGo <= TURN_TOLERANCE_IN_DEGREES)
                break;

            this.curDrivePower = pidController.getTurnPower(this.curDrivePower, degressToGo, targetPower, rampDown);

            motorFR.setPower( -1 * this.curDrivePower );
            motorFL.setPower(      this.curDrivePower );
            motorBR.setPower( -1 * this.curDrivePower );
            motorBL.setPower(      this.curDrivePower );
        }

        if (stopMotors) {
            stop();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // turnLeftTillDegrees
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void turnLeftTillDegrees( int targetDegrees, double targetPower, boolean rampDown, boolean stopMotors ) {
        double currentHeading   = 0;
        double degressToGo      = 0;

        while (robot.continueMotion()) {
            this.clearBulkCache();

            currentHeading = getCurrentPositionInDegreesUsingOdometry();

            degressToGo = currentHeading - targetDegrees;

            if (degressToGo < -120)
                degressToGo += 360;

            if (degressToGo <= TURN_TOLERANCE_IN_DEGREES)
                break;

            this.curDrivePower = pidController.getTurnPower(this.curDrivePower, degressToGo, targetPower, rampDown);

            motorFR.setPower(      this.curDrivePower );
            motorFL.setPower( -1 * this.curDrivePower );
            motorBR.setPower(      this.curDrivePower );
            motorBL.setPower( -1 * this.curDrivePower );
        }

        if (stopMotors) {
            stop();
        }
    }

}
