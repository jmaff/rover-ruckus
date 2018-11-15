package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc12835.library.hardware.management.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * Created by 21maffetone on 11/13/18.
 */

public class Vision implements Subsystem {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AfqJicT/////AAABmV0Cv8mz10n/joVc93wk6uxXh5eJuApDAXck9Oc1MWCIiRkjrE47UP6ODjidREdy5b3zc1HGJT3/B82inSlLRCa4WpjzbZvHl5FOslDg1EfWtHJH7BhXW8hVVXZJvSSwwmEP8jRWXFudvIMF5QxvVUxcj60RaL1amo00qbROMK5OOJrfDIj4Knbox6pJX17d0WDwJ66XqPbtCQstlPyHSr/iGH81fxLa0cr2Gvk5YiWHNUI4drOz7yPvFzR5Ja7J+MUagIbqtCWmWeUjgEcKqIZw0C4rfJbBsVAJ+Zx58q96hng3GgbXqZLJMcnK1kqOgNXm874Mm+XUQLMWsIIhwBMoMzg0hp5jlDxI0+2XDYGL";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private OpMode opMode;

    public Vision(OpMode opMode, boolean auto) {
        this.opMode = opMode;
        if (auto) {
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "SAMPLING_CAMERA");

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                int tfodMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier(
                        "tfodMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
                TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
                tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
                tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
            } else {
                opMode.telemetry.addData("Sorry!", "This device is not compatible with TFOD");
            }
        }
    }

    public void enable() {
        tfod.activate();
    }

    @Override
    public void update() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            opMode.telemetry.addData("Gold Mineral Position", "Left");
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            opMode.telemetry.addData("Gold Mineral Position", "Right");
                        } else {
                            opMode.telemetry.addData("Gold Mineral Position", "Center");
                        }
                    }
                }

                opMode.telemetry.update();
            }
        }
    }

    public void disable() {
        tfod.deactivate();
    }
}
