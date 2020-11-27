package org.firstinspires.ftc.teamcode.ultimategoal.robot;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.KalmanFilter;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.PIDController;

import java.util.List;

public class Intake {

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare motors variables
    ////////////////////////////////////////////////////////////////////////////////////
    DcMotorEx       motorIntake;
    DcMotorEx       motorIntakeLift;

    //////////////////////////////////////////
    Servo           servoBasketRight;
    Servo           servoBasketLeft;
    // Declare reference to main robot
    //////////////////////////////////////////
    Robot               robot;

    //----------------------------------------------------------------------------------------------------------------------------------
    // Class Methods Starts Here
    //----------------------------------------------------------------------------------------------------------------------------------

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Constructor
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Intake(Robot robot ) {
        this.robot          = robot;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // initDriveMotors
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void init() {
        motorIntake = robot.opMode.hardwareMap.get(DcMotorEx.class, "motorIntake");  // Configure the robot to use these 4 motor names,
        motorIntake.setDirection(DcMotor.Direction.REVERSE);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorIntakeLift = robot.opMode.hardwareMap.get(DcMotorEx.class, "motorIntakeLift");  // Configure the robot to use these 4 motor names,
        motorIntakeLift.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.servoBasketRight      = robot.opMode.hardwareMap.get(Servo.class, "servoBasketRight");
        this.servoBasketLeft      = robot.opMode.hardwareMap.get(Servo.class, "servoBasketLeft");



    }

    public void intake() {
        motorIntake.setPower(0.5);
        motorIntakeLift.setPower(0.2);
    }

    public void outake() {
        motorIntake.setPower(-0.5);
        motorIntakeLift.setPower(0.25);
    }

    public void turnOff() {
        motorIntake.setPower(0.0);
        motorIntakeLift.setPower(0.0);
    }

    public void liftBasket() {
        double pos = 0.20;
        servoBasketRight.setPosition(pos);
//        servoBasketLeft.setPosition(1-pos);
    }

    public void lowerBasket() {
        double pos = 0.7;
        servoBasketRight.setPosition(pos);
//        servoBasketLeft.setPosition(1-pos);
    }

}
