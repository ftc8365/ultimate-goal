package org.firstinspires.ftc.teamcode.ultimategoal.robot;


import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

import java.util.List;

public class Robot {

    /////////////////////
    // Declare constants
    /////////////////////

    public final int NEVEREST_20_TICKS_PER_REV   = 538;
    public final int NEVEREST_20_MAX_RPM         = 340;

    public final double ODOMETRY_WHEEL_RADIUS            = 0.74803;
    public final int    ODOMETRY_WHEEL_TICK_PER_ROTATION = 1440;

    final public int TICK_PER_WHEEL_ROTATION   = NEVEREST_20_TICKS_PER_REV;

    final int AUTONOMOUS_DURATION_MSEC  = 29800;

    final double INTAKE_POWER           = 0.35;
    final double RAMP_UP_RATE_DRIVE     = 0.01;
    final double RAMP_UP_RATE_TURN      = 0.01;
    final double RAMP_UP_RATE_STRAFE    = 0.05;

    final double RAMP_DOWN_DRIVE_TICKS  = 150;
    final double RAMP_DOWN_DRIVE_ODOMETRY_TICKS  = 150;

    final double RAMP_DOWN_DRIVE_RANGE  = 10;
    final double RAMP_DOWN_TURN_DEGREES = 30;
    final double RAMP_DOWN_STRAFE_TICKS = 150;

    final double MIN_DRIVE_POWER        = 0.10;
    final double MIN_TURN_POWER         = 0.35;
    final double MIN_STRAFE_POWER       = 0.20;
    final double TURN_POWER             = 0.70;
    final double TURN_TOLERANCE         = 5.0;

    boolean runningAutonomous           = true;
    AllianceMode allianceMode           = AllianceMode.ALLIANCE_BLUE;

    public enum AllianceMode {
        ALLIANCE_BLUE,
        ALLIANCE_RED
    }

    /////////////////////
    // Declare motors variables
    /////////////////////
    public DcMotor motorFR = null;
    public DcMotor motorFL = null;
    public DcMotor motorBR = null;
    public DcMotor motorBL = null;

    private PIDController pidController = new PIDController();
    private KalmanFilter kalmanFilter = new KalmanFilter();
    private TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder();

    /////////////////////
    // Declare vuforia tensorflow variables
    /////////////////////

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AaaD61H/////AAABmfJ7OgkkWEWVmniO8RAqZ1cEkXF6bR9ebw4Gw+hUI8s1s5iTA9Hyri+sjoSO/ISwSWxfZCI/iAzZ0RxSQyGQ7xjWaoE4AJgn4pKLKVcOsuglHJQhjQevzdFKWX6cXq4xYL6vzwX7G7zuUP6Iw3/TzZIAj7OxYl49mA30JfoXvq/kKnhDOdM531dbRyZiaNwTGibRl5Dxd4urQ5av3EU1QyFBWR04eKWBrJGffk8bdqjAbB3QDp/7ZAMi2WfkItMOP5ghc5arNRCNt5x+xX7gSq8pMt3ZoC3XPfRNNaEbf24MgaNJXlUARsfAuycgPiY83jbX0Hrctj4wZ20wqah+FNYMqvySokw6/fDmyG0mPmel";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    /////////////////////
    // Declare sensors
    /////////////////////
    /////////////////////
    IntegratingGyroscope            gyro            = null;
    NavxMicroNavigationSensor       navxMicro       = null;

    ElapsedTime autonomusTimer                      = new ElapsedTime();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Declare Servos
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Servo servoExample         = null;

    LinearOpMode opMode         = null;

    public void setOpMode(LinearOpMode opMode ) {
        this.opMode = opMode;
    }

    //----------------------------------------------------------------------------------------------
    // Initialization Methods
    //----------------------------------------------------------------------------------------------

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
        motorFR = opMode.hardwareMap.get(DcMotor.class, "motorFR");
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL  = opMode.hardwareMap.get(DcMotor.class, "motorFL");
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBR = opMode.hardwareMap.get(DcMotor.class, "motorBR");
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBL  = opMode.hardwareMap.get(DcMotor.class, "motorBL");
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        navxMicro = opMode.hardwareMap.get(NavxMicroNavigationSensor.class, "gyro_sensor");
        gyro = (IntegratingGyroscope)navxMicro;

        while (navxMicro.isCalibrating())  {
            opMode.sleep(50);
        }
    }

    public void initTensorFlowObjectDetectionWebcam() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforiaWebcam();

        initTfod();

        if (tfod != null) {
            tfod.activate();
        }
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


    public List<Recognition> getTensorFlowRecognitions() {
        if (tfod != null) {

            List<Recognition> updatedRecognitions = tfod.getRecognitions();

            if (updatedRecognitions != null) {
                return updatedRecognitions;
            }
        }
        return null;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initVuforiaWebcam() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = this.opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void shutdownTensorFlow() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void setAllianceMode( AllianceMode mode ) {
        this.allianceMode = mode;
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


//    float getCurrentHeading() {
//        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

//        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) * -1;
//    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // setDriveMotorPower
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void setDriveMotorPower(double powerFL, double powerFR, double powerBL, double powerBR) {
        this.motorFL.setPower( powerFL );
        this.motorFR.setPower( powerFR );
        this.motorBL.setPower( powerBL );
        this.motorBR.setPower( powerBR );
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
            return ( opMode.opModeIsActive() ) && ( autonomusTimer.milliseconds() < AUTONOMOUS_DURATION_MSEC );
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveForwardTillDistance
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForwardTillDistance( double distanceInches, double initPower, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {

        double inchesPerRotation = 2 * Math.PI * ODOMETRY_WHEEL_RADIUS;

        double ticks = (distanceInches / inchesPerRotation) * ODOMETRY_WHEEL_TICK_PER_ROTATION;

        driveForwardTillTicks((int)ticks, initPower, targetPower,targetHeading, rampDown, stopMotors );
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveForwardTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForwardTillTicks( int ticks, double initPower, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors )
    {
        boolean useGyroToAlign  = (this.gyro != null && targetHeading >= 0) ? true : false;
        int initPosition        = motorFL.getCurrentPosition();
        int ticksToGo           = 0;
        double power            = initPower;

        while (continueAutonomus()) {
            ticksToGo = ticks - (motorFL.getCurrentPosition() - initPosition);

            if (ticksToGo <= 0)
                break;

            //get power based on distance
            power = pidController.getDrivePower(power, ticksToGo, targetPower, rampDown);

            double powerRight = power;
            double powerLeft  = power;

            // Adjusting motor power based on gyro position
            // to force the robot to move straight
            if (useGyroToAlign) {
                double currentPosition = this.getCurrentPositionInDegrees();
                double headingChange   = currentPosition - targetHeading;

                if (headingChange > 180 && targetHeading <= 30) {
                    headingChange -= 360;
                }

                powerRight +=  2 * (headingChange / 100);
                powerLeft  -=  2 * (headingChange / 100);
            }

            motorFR.setPower( powerRight );
            motorFL.setPower( powerLeft );
            motorBR.setPower( powerRight );
            motorBL.setPower( powerLeft );

        }

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }

    private void setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motorFR.setZeroPowerBehavior(behavior);
        motorFL.setZeroPowerBehavior(behavior);
        motorBR.setZeroPowerBehavior(behavior);
        motorBL.setZeroPowerBehavior(behavior);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveLeftDiagionalTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveLeftDiagionalTillRotation( double distanceInInches, double targetPower, boolean rampDown, boolean stopMotors ) {

        double inchesPerRotation = 2 * Math.PI * ODOMETRY_WHEEL_RADIUS;

        double ticks = (distanceInInches / inchesPerRotation) * ODOMETRY_WHEEL_TICK_PER_ROTATION;

        driveLeftDiagionalTillTicks( (int)ticks, targetPower, rampDown, stopMotors );
    }

    public void driveLeftDiagionalTillTicks( int ticks, double targetPower, boolean rampDown, boolean stopMotors )
    {
        int initPosition        = motorFL.getCurrentPosition();
        int ticksToGo           = 0;
        double power            = 0.30;

        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (continueAutonomus()) {

            ticksToGo = ticks - (motorFL.getCurrentPosition() - initPosition);

            if (ticksToGo <= 0)
                break;

            power = pidController.getDrivePower(power, ticksToGo, targetPower, rampDown);

            motorFR.setPower( power );
            motorFL.setPower( 0 );
            motorBR.setPower( 0 );
            motorBL.setPower( power );
        }

        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveRightDiagionalTillRotation
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveRightDiagionalTillRotation( double rotation, double targetPower, boolean rampDown, boolean stopMotors )
    {
        int initPosition        = motorFR.getCurrentPosition();
        int ticksToGo           = 0;
        double power            = 0.0;

        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (continueAutonomus()) {

            ticksToGo = (int)(TICK_PER_WHEEL_ROTATION * rotation) - (motorFR.getCurrentPosition() - initPosition);

            if (ticksToGo <= 0)
                break;

            power = pidController.getDrivePower(power, ticksToGo, targetPower, rampDown);

            motorFR.setPower( 0 );
            motorFL.setPower( power );
            motorBR.setPower( power );
            motorBL.setPower( 0 );
        }

        setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private int getLeftOdometryPostion() {
        return motorBR.getCurrentPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private int getRightOdometryPostion() {
        return motorBR.getCurrentPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // getOdometryPosition
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private int getCenterOdometryPosition() {
        return motorBL.getCurrentPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // driveFBackwardTillDistance
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackwardTillDistance( double distanceInches, double initPower, double targetPower, int targetHeading, boolean rampDown, boolean stopMotors ) {

        double inchesPerRotation = 2 * Math.PI * ODOMETRY_WHEEL_RADIUS;

        double ticks = (distanceInches / inchesPerRotation) * ODOMETRY_WHEEL_TICK_PER_ROTATION;

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

            if (maxTime > 0) {
                if ((autonomusTimer.milliseconds() - startingTime) > maxTime)
                    break;
            }

            power = pidController.getDrivePower(power, ticksToGo, targetPower, rampDown);

            if (rampDownRange != -1 && ticksToGo < rampDownRange)
                power = MIN_DRIVE_POWER;

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
    public void turnRightTillDegrees( int targetDegrees, boolean rampDown, boolean stopMotors ) {
        turnRightTillDegrees(targetDegrees, TURN_POWER, rampDown,stopMotors);
    }

    public void turnRightTillDegrees( int targetDegrees, double targetPower, boolean rampDown, boolean stopMotors )
    {
        double power = 0.30;
        double currentHeading = 0;
        double degressToGo = 0;

        while (continueAutonomus()) {

            currentHeading = getCurrentPositionInDegrees();
            if (currentHeading > 180 && targetDegrees < 180)
                currentHeading -= 360;

            degressToGo = targetDegrees - currentHeading;
            if (degressToGo <= TURN_TOLERANCE)
                break;

            power = this.getTurnPower(power, degressToGo, targetPower, rampDown);

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
    public void turnLeftTillDegrees( int targetDegrees, boolean rampDown, boolean stopMotors ) {
        turnLeftTillDegrees(targetDegrees, TURN_POWER, rampDown,stopMotors);
    }

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

            if (degressToGo <= TURN_TOLERANCE)
                break;

            power = this.getTurnPower(power, degressToGo, targetPower, rampDown);

            motorFR.setPower(      power );
            motorFL.setPower( -1 * power );
            motorBR.setPower(      power );
            motorBL.setPower( -1 * power );
        }

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // turnLeftTillDegrees
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void curveLeftTillDegrees( int targetDegrees, boolean rampDown, boolean stopMotors )
    {
        double targetPower = 0.20;
        double power = 0.05;
        double currentHeading = 0;
        double degressToGo = 0;

        while (continueAutonomus()) {

            currentHeading = getCurrentPositionInDegrees();
            if (currentHeading < 90) {
                currentHeading += 360;
            }

            if (currentHeading > 270 && targetDegrees < 90) {
                targetDegrees += 360;
            }

            degressToGo = currentHeading - targetDegrees;

            if (degressToGo <= TURN_TOLERANCE)
                break;

            power = this.getTurnPower(power, degressToGo, targetPower, rampDown);

            motorFR.setPower( power * 1.5 );
            motorFL.setPower( power * 0.5);
            motorBR.setPower( power * 1.5 );
            motorBL.setPower( power * 0.5 );
        }

        if (stopMotors) {
            this.stopDriveMotors();
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // getTurnPower
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private double getTurnPower( double curPower, double degreesToGo, double targetPower, boolean rampDown ) {
        double power = curPower;
        double RAMP_DOWN_DISTANCE = 30;

        // Ramp down power
        if ( rampDown && (degreesToGo <= RAMP_DOWN_DISTANCE) ) {
            power = ( degreesToGo / RAMP_DOWN_DISTANCE)  * curPower;

            if (power < MIN_TURN_POWER)
                power = MIN_TURN_POWER;
        }
        else if (targetPower - power > 0.001) {
            power += this.RAMP_UP_RATE_TURN;
        }
        return power;
    }



    public TrajectoryBuilder trajectoryBuilder() {
        return this.trajectoryBuilder;
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
                                                movement.getConstraint().getRampDown(),
                                                movement.getConstraint().getStopMotor() );
                    break;
                case TURN_RIGHT:
                    this.turnRightTillDegrees( movement.getTargetHeading(),
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
