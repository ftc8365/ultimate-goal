package org.firstinspires.ftc.teamcode.ultimategoal.robot;


import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.KalmanFilter;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.PIDController;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.Trajectory;
import org.firstinspires.ftc.teamcode.ultimategoal.robot.motionprofiling.TrajectoryBuilder;

import java.util.List;

public class Grabber {

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare Motor
    ////////////////////////////////////////////////////////////////////////////////////
    DcMotorEx       motorGrabberArm;

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare Servos
    ////////////////////////////////////////////////////////////////////////////////////
    Servo           servoGrabber;

    ////////////////////////////////////////////////////////////////////////////////////
    // Declare Reference to robot
    ////////////////////////////////////////////////////////////////////////////////////
    Robot           robot;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Constructor
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Grabber(Robot robot ) {
        this.robot = robot;
    }

    public void init() {
        this.servoGrabber = robot.opMode.hardwareMap.get(Servo.class, "servoGrabber");

        // TODO : initialize motor for grabber arm
    }

    // TODO : turn servo to open Grabber
    public void openGrabber() {

    }

    // TODO : turn servo to close Grabber
    public void closeGrabber() {

    }



}
