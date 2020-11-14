package org.firstinspires.ftc.teamcode.ultimategoal.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Shooter {

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare motors variables
    ////////////////////////////////////////////////////////////////////////////////////
    DcMotorEx       motorShooter;

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
    public Shooter(Robot robot ) {
        this.robot = robot;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // initDriveMotors
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        public void init() {
            motorShooter = robot.opMode.hardwareMap.get(DcMotorEx.class, "motorShooter");  // Configure the robot to use these 4 motor names,
            motorShooter.setDirection(DcMotor.Direction.FORWARD);
            motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void shooterOn(double targetVelocity){
            motorShooter.setVelocity(targetVelocity);
        }

        public double getVelocity() {
            return motorShooter.getVelocity();
        }

        public void shooterOff(){
            motorShooter.setPower(0);
        }

}
