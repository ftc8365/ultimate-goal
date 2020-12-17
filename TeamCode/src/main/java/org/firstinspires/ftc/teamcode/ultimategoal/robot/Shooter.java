package org.firstinspires.ftc.teamcode.ultimategoal.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter {

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare motors variables
    ////////////////////////////////////////////////////////////////////////////////////
    DcMotorEx       motorShooter;
    Servo           servoPoker;
    Servo           servoPokerAssist;

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
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

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

        servoPoker = robot.opMode.hardwareMap.get(Servo.class, "servoPoker");

        servoPokerAssist = robot.opMode.hardwareMap.get(Servo.class, "servoPokerAssist");

        // Increase value to lower tilt
        // Decrease value to raise tilt
        servoPokerAssist.setPosition(0.58);
    }

    public void initShooterTest() {

        motorShooter = robot.opMode.hardwareMap.get(DcMotorEx.class, "motorShooter");  // Configure the robot to use these 4 motor names,
        motorShooter.setDirection(DcMotor.Direction.REVERSE);
        motorShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.servoPoker = robot.opMode.hardwareMap.get(Servo.class, "servoPoker");
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

            case SHOOTER_STATE_3:
                motorShooter.setVelocity(500);
                break;

            case SHOOTER_STATE_4:
                motorShooter.setVelocity(1500);
                break;
        }
    }

    public void stop() {
        motorShooter.setPower(0);
    }

    public double getPower() {
        return this.motorShooter.getPower();
    }

    public double getVelocity() {
        return this.motorShooter.getVelocity();
    }

    public void pushPoker() {
        servoPoker.setPosition( 0.45 );
    }

    public void stopPoker() {
        servoPoker.setPosition( 0.75 );
    }

    public void pushPokerTest() {
        servoPoker.setPosition( 0.3 );
    }

    public void stopPokerTest() {
        servoPoker.setPosition( 0.35 );
    }


    public void burstPoker() {
        // 3 to 1 gear ration is 33 milli
        pushPoker();
        sleep(150);
        stopPoker();
        sleep(150);
        pushPoker();
        sleep(150);
        stopPoker();
        sleep(150);
        pushPoker();
        sleep(150);
        stopPoker();
    }

}
