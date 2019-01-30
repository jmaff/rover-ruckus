package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class Vision implements Subsystem {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AfqJicT/////AAABmV0Cv8mz10n/joVc93wk6uxXh5eJuApDAXck9Oc1MWCIiRkjrE47UP6ODjidREdy5b3zc1HGJT3/B82inSlLRCa4WpjzbZvHl5FOslDg1EfWtHJH7BhXW8hVVXZJvSSwwmEP8jRWXFudvIMF5QxvVUxcj60RaL1amo00qbROMK5OOJrfDIj4Knbox6pJX17d0WDwJ66XqPbtCQstlPyHSr/iGH81fxLa0cr2Gvk5YiWHNUI4drOz7yPvFzR5Ja7J+MUagIbqtCWmWeUjgEcKqIZw0C4rfJbBsVAJ+Zx58q96hng3GgbXqZLJMcnK1kqOgNXm874Mm+XUQLMWsIIhwBMoMzg0hp5jlDxI0+2XDYGL";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public static int CENTER_THRESHOLD = 500;

    private OpMode opMode;
    public enum GoldPosition {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN
    }

    private double goldWidth = 0;
    private double goldTop = 0;
    private double imageHeight = 0;
    private double goldPos;
    private int numMinerals = 0;

    private int leftCount = 0;
    private int centerCount = 0;
    private int rightCount = 0;

    private GoldPosition goldPosition = GoldPosition.UNKNOWN;

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
                tfodParameters.minimumConfidence = 0.46;
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

    public GoldPosition getGoldPosition() { return goldPosition; }

    @Override
    public void update() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                numMinerals = updatedRecognitions.size();
                if (updatedRecognitions.size() >= 0) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                            goldWidth = recognition.getRight() - recognition.getLeft();
                            goldTop = recognition.getTop();
                            imageHeight = recognition.getImageHeight();
                            if (goldWidth < 250 && goldTop < imageHeight/2.0) {
                                goldPos = (int) ((double) recognition.getLeft()+(double)recognition.getRight()) / 2.0;
                            }
                        }
                    }

                    if (goldPos != -1) {
                        if (goldPos < CENTER_THRESHOLD) {
                            centerCount++;
                        } else {
                            rightCount++;
                        }
                    } else {
                        leftCount++;
                    }

                    if (leftCount > rightCount && leftCount > centerCount) {
                        goldPosition = GoldPosition.LEFT;
                    } else if (rightCount > leftCount && rightCount > centerCount) {
                        goldPosition = GoldPosition.RIGHT;
                    } else {
                        goldPosition = GoldPosition.CENTER;
                    }


//                    for (Recognition recognition : updatedRecognitions) {
//                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                            goldMineralX = (int) recognition.getLeft();
//                        } else if (silverMineral1X == -1) {
//                            silverMineral1X = (int) recognition.getLeft();
//                        }
//                    }


//
//                    if (goldMineralX == -1) {
//                        goldPosition = GoldPosition.LEFT;
//                    } else if (silverMineral1X != -1) {
//                        if (goldMineralX < silverMineral1X) {
//                            goldPosition = GoldPosition.CENTER;
//                        } else {
//                            goldPosition = GoldPosition.RIGHT;
//                        }
//                    }
                }

            }
        }
    }

    public double getGoldPos() {
        return goldPos;
    }

    public int getNumMinerals() {
        return numMinerals;
    }

    public void disable() {
        tfod.deactivate();
    }
}
