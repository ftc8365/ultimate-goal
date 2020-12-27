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
    // Declare servo variables
    ////////////////////////////////////////////////////////////////////////////////////
    Servo           servoBasketRight;

    //////////////////////////////////////////
    // Declare reference to main robot
    //////////////////////////////////////////
    Robot           robot;
    boolean         basketUp;
    //----------------------------------------------------------------------------------------------------------------------------------
    // Class Methods Starts Here
    //----------------------------------------------------------------------------------------------------------------------------------

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Constructor
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Intake(Robot robot ) {
        this.robot          = robot;
        this.basketUp       = false;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // initDriveMotors
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void init() {
        motorIntake = robot.opMode.hardwareMap.get(DcMotorEx.class, "motorIntake");
        motorIntake.setDirection(DcMotor.Direction.REVERSE);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorIntakeLift = robot.opMode.hardwareMap.get(DcMotorEx.class, "motorIntakeLift");
        motorIntakeLift.setDirection(DcMotor.Direction.REVERSE);
        motorIntakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.servoBasketRight      = robot.opMode.hardwareMap.get(Servo.class, "servoBasketRight");
    }

    public void stop() {
        motorIntake.setPower(0.0);
        motorIntakeLift.setPower(0.0);
    }

    public void intake() {
        motorIntake.setPower(0.25);
        motorIntakeLift.setPower(0.45);
//        motorIntake.setVelocity(480);
//        motorIntakeLift.setVelocity(1280);
    }

    public void outake() {
        motorIntake.setPower(-0.5);
        motorIntakeLift.setPower(0.25);
    }

    public void liftBasket() {
        double pos = 0.780;
        basketUp = true;
        servoBasketRight.setPosition(pos);
    }

    public void lowerBasket() {
        double pos = 0.07;
        basketUp = false;
        servoBasketRight.setPosition(pos);
    }

    public boolean isBasketUp() {
        return basketUp;
    }

    public double getIntakeVelocity() {
        return this.motorIntake.getVelocity();
    }
    public double getIntakeLiftVelocity() {
        return this.motorIntakeLift.getVelocity();
    }

    public double getIntakePosition() {
        return this.motorIntake.getCurrentPosition();
    }
    public double getIntakeLiftPosition() {
        return this.motorIntakeLift.getCurrentPosition();
    }

}
