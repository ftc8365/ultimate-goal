package org.firstinspires.ftc.teamcode.ultimategoal.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Grabber {

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare Motor
    ////////////////////////////////////////////////////////////////////////////////////

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
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void armUp() {
        double pos  = 0.1;
        servoArm1.setPosition( pos );
        servoArm2.setPosition( 1 - pos );
    }

    public void armDown() {
        double pos = 0.50;
        servoArm1.setPosition(pos);
        servoArm2.setPosition(1-pos);
    }

    public void armDownAuto() {
        double pos  = 0.1;
        servoArm1.setPosition( pos );
        servoArm2.setPosition( 1 - pos );
    }

    public void armUpAuto() {
        double pos = 0.75;
        servoArm1.setPosition(pos);
        servoArm2.setPosition(1-pos);
    }

    public void openGrabber() {
        servoGrabber.setPosition(1);
    }

    public void closeGrabber() {
        servoGrabber.setPosition(0.55);
    }

}
