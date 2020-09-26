package org.firstinspires.ftc.teamcode.ultimategoal.robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


public class ComputerVision {
    final Telemetry telemetry;
    final HardwareMap hardwareMap;

    // Declare VUFORIA KEY
    private static final String VUFORIA_KEY =
            "AaaD61H/////AAABmfJ7OgkkWEWVmniO8RAqZ1cEkXF6bR9ebw4Gw+hUI8s1s5iTA9Hyri+sjoSO/ISwSWxfZCI/iAzZ0RxSQyGQ7xjWaoE4AJgn4pKLKVcOsuglHJQhjQevzdFKWX6cXq4xYL6vzwX7G7zuUP6Iw3/TzZIAj7OxYl49mA30JfoXvq/kKnhDOdM531dbRyZiaNwTGibRl5Dxd4urQ5av3EU1QyFBWR04eKWBrJGffk8bdqjAbB3QDp/7ZAMi2WfkItMOP5ghc5arNRCNt5x+xX7gSq8pMt3ZoC3XPfRNNaEbf24MgaNJXlUARsfAuycgPiY83jbX0Hrctj4wZ20wqah+FNYMqvySokw6/fDmyG0mPmel";

    // TODO : Declare Vurforia & TensorFlow Variables


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    public ComputerVision(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO
    public void initVuforia() {
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO
    public void initTfod() {
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO
    public void activate() {
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO
    public void detect() {
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO
    public void shutdown() {
    }

}
