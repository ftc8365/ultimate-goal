package org.firstinspires.ftc.teamcode.preseason2020;


import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

public class Robot {

    /////////////////////
    // Declare constants
    /////////////////////



    /////////////////////
    // RevExapansionHubEx
    /////////////////////
    RevBulkData bulkData;
    AnalogInput a0, a1, a2, a3;
    DigitalChannel d0, d1, d2, d3, d4, d5, d6, d7;
    public ExpansionHubEx expansionHub;

    /////////////////////
    // Declare motors variables
    /////////////////////

    public ExpansionHubMotor motorFR = null;
    public ExpansionHubMotor motorFL = null;
    public ExpansionHubMotor motorBR = null;
    public ExpansionHubMotor motorBL = null;

    /////////////////////
    // Declare sensors
    /////////////////////
    IntegratingGyroscope            gyro            = null;
    NavxMicroNavigationSensor       navxMicro       = null;


    public Telemetry telemetry      = null;
    public HardwareMap hardwareMap  = null;


    //----------------------------------------------------------------------------------------------
    // Initialization Methods
    //----------------------------------------------------------------------------------------------
    public void setHardwareMap( HardwareMap hardwareMap ) {
        this.hardwareMap = hardwareMap;
    }

    public void setTelemetry( Telemetry telemetry ) {
        this.telemetry = telemetry;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // initDriveMotors
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void initDriveMotors() {

        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        motorFR = (ExpansionHubMotor) hardwareMap.dcMotor.get("motorFR");
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL = (ExpansionHubMotor) hardwareMap.dcMotor.get("motorFL");
        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBR = (ExpansionHubMotor) hardwareMap.dcMotor.get("motorBR");
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBL = (ExpansionHubMotor) hardwareMap.dcMotor.get("motorBL");
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // initGyroSensor
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void initGyroSensor() {
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "gyro_sensor");
        gyro = (IntegratingGyroscope)navxMicro;

        while (navxMicro.isCalibrating())  {
            try {
                Thread.sleep(50);
            } catch (Exception e) {
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // getCurrentPositionInDegrees
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // getCurrentHeading
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double getCurrentHeading() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle) * -1;
    }


    public void getInputData() {
        bulkData = expansionHub.getBulkInputData();
    }

    public int getMotorFRPosition() {
//        return bulkData.getMotorCurrentPosition(motorFR);
        return bulkData.getMotorCurrentPosition(0);
    }
    public int getMotorFLPosition() {
//        return bulkData.getMotorCurrentPosition(motorFL);
        return bulkData.getMotorCurrentPosition(1);
    }
    public int getMotorBRPosition() {
//        return bulkData.getMotorCurrentPosition(motorBR);
        return bulkData.getMotorCurrentPosition(2);
    }
    public int getMotorBLPosition() {
//        return bulkData.getMotorCurrentPosition(motorBL);
        return bulkData.getMotorCurrentPosition(3);
    }

    public int getMotorFRVelocity() {
        return bulkData.getMotorVelocity(0);
    }
    public int getMotorFLVelocity() {
        return bulkData.getMotorVelocity(1);
    }
    public int getMotorBRVelocity() {
        return bulkData.getMotorVelocity(2);
    }
    public int getMotorBLVelocity() {
        return bulkData.getMotorVelocity(3);
    }

    public void stopAllMotors() {
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
    }

}