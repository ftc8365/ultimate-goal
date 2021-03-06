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

    final Robot robot;

    // Declare VUFORIA KEY
    private static final String VUFORIA_KEY =
            "AaaD61H/////AAABmfJ7OgkkWEWVmniO8RAqZ1cEkXF6bR9ebw4Gw+hUI8s1s5iTA9Hyri+sjoSO/ISwSWxfZCI/iAzZ0RxSQyGQ7xjWaoE4AJgn4pKLKVcOsuglHJQhjQevzdFKWX6cXq4xYL6vzwX7G7zuUP6Iw3/TzZIAj7OxYl49mA30JfoXvq/kKnhDOdM531dbRyZiaNwTGibRl5Dxd4urQ5av3EU1QyFBWR04eKWBrJGffk8bdqjAbB3QDp/7ZAMi2WfkItMOP5ghc5arNRCNt5x+xX7gSq8pMt3ZoC3XPfRNNaEbf24MgaNJXlUARsfAuycgPiY83jbX0Hrctj4wZ20wqah+FNYMqvySokw6/fDmyG0mPmel";

    // Declare Vurforia & TensorFlow Variables
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String LABEL_NO_ELEMENT = "None";

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    public ComputerVision(Robot robot) {
        this.robot = robot;
    }

    public void init() {
        initVuforia();
        initTfod();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = robot.opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void initTfod() {
        int tfodMonitorViewId = robot.opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", robot.opMode.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void activate() {
//        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // TODO: change the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(3, 1.777);

//        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    public String detect() {
        String value = "None";
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> recognitions = tfod.getRecognitions();

            if (recognitions != null) {
                robot.opMode.telemetry.addData("# Object Detected", recognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : recognitions) {
                    robot.opMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    robot.opMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                    robot.opMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());
                    value = recognition.getLabel();
                }
            }
        }
        return value;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void stop() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

}
