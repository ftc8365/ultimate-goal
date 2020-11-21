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
    Robot           robot;
    ShooterState    state;

    public enum ShooterState {
        SHOOTER_OFF,
        SHOOTER_STATE_1,
        SHOOTER_STATE_2,
        SHOOTER_STATE_3,
        SHOOTER_STATE_4
    }

    //----------------------------------------------------------------------------------------------------------------------------------
    // Class Methods Starts Here
    //----------------------------------------------------------------------------------------------------------------------------------

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Constructor
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Shooter(Robot robot ) {
        this.robot = robot;
        this.state = ShooterState.SHOOTER_OFF;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // initDriveMotors
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        public void init() {
            motorShooter = robot.opMode.hardwareMap.get(DcMotorEx.class, "motorShooter");  // Configure the robot to use these 4 motor names,
            motorShooter.setDirection(DcMotor.Direction.FORWARD);
            motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        public void setState( ShooterState state ) {
            this.state = state;

            switch (state) {
                case SHOOTER_OFF:
                    motorShooter.setPower(0);
                    break;

                case SHOOTER_STATE_1:
                    motorShooter.setVelocity(1800);
                    break;

                case SHOOTER_STATE_2:
                    motorShooter.setVelocity(100);
                    break;
            }
        }

        public void stop() {
            setState( ShooterState.SHOOTER_OFF );
        }

/*        public void shooterOn(double targetVelocity) {
            motorShooter.setVelocity(targetVelocity);
        }

        public double getVelocity() {
            return motorShooter.getVelocity();
        }

*/
}
