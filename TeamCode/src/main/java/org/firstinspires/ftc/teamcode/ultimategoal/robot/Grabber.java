package org.firstinspires.ftc.teamcode.ultimategoal.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare Motor
    ////////////////////////////////////////////////////////////////////////////////////
    DcMotorEx       motorGrabberArm;

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare Servos
    ////////////////////////////////////////////////////////////////////////////////////
    Servo           servoGrabber;

    Servo           servoArm1;
    Servo           servoArm2;

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare Reference to robot
    ////////////////////////////////////////////////////////////////////////////////////
    Robot           robot;

    enum ArmState {
        ARM_STATE_1,
        ARM_STATE_2,
        ARM_STATE3
    };

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Constructor
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Grabber(Robot robot ) {
        this.robot = robot;
    }

    public void init() {
        this.servoGrabber   = robot.opMode.hardwareMap.get(Servo.class, "servoGrabber");
        this.servoArm1      = robot.opMode.hardwareMap.get(Servo.class, "servoGrabberArm1");
        this.servoArm2      = robot.opMode.hardwareMap.get(Servo.class, "servoGrabberArm2");

        motorGrabberArm = robot.opMode.hardwareMap.get(DcMotorEx.class, "motorGrabberArm");  // Configure the robot to use these 4 motor names,
        motorGrabberArm.setDirection(DcMotor.Direction.FORWARD);
        motorGrabberArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void armUp() {
        motorGrabberArm.setPower(0.3);

        try {
            Thread.sleep(250);
        } catch (Exception e) {
        }
        motorGrabberArm.setPower(0);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void armDown(){

        motorGrabberArm.setPower(-0.3);
        try {
            Thread.sleep(250);
        } catch (Exception e) {
        }
        motorGrabberArm.setPower(0);
    }

    public void openGrabber() {
        servoGrabber.setPosition(1);
    }

    public void closeGrabber() {
        servoGrabber.setPosition(0.55);
    }



}
