package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.ftc12835.library.hardware.management.Subsystem;
import com.ftc12835.library.vision.VuforiaCamera;
import com.ftc12835.roverruckus.vision.SamplingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
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
    public static int CENTER_THRESHOLD = 400;
    private static final String VUFORIA_KEY = "AfqJicT/////AAABmV0Cv8mz10n/joVc93wk6uxXh5eJuApDAXck9Oc1MWCIiRkjrE47UP6ODjidREdy5b3zc1HGJT3/B82inSlLRCa4WpjzbZvHl5FOslDg1EfWtHJH7BhXW8hVVXZJvSSwwmEP8jRWXFudvIMF5QxvVUxcj60RaL1amo00qbROMK5OOJrfDIj4Knbox6pJX17d0WDwJ66XqPbtCQstlPyHSr/iGH81fxLa0cr2Gvk5YiWHNUI4drOz7yPvFzR5Ja7J+MUagIbqtCWmWeUjgEcKqIZw0C4rfJbBsVAJ+Zx58q96hng3GgbXqZLJMcnK1kqOgNXm874Mm+XUQLMWsIIhwBMoMzg0hp5jlDxI0+2XDYGL";

    public VuforiaCamera camera;
    public SamplingPipeline pipeline;

    private GoldPosition goldPosition;

    private OpMode opMode;
    public enum GoldPosition {
        LEFT,
        CENTER,
        RIGHT,
        UNKNOWN,
        ERROR
    }

    private boolean active = false;


    public Vision(OpMode opMode, boolean auto) {
        this.opMode = opMode;
        if (auto) {
            VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
            params.vuforiaLicenseKey = VUFORIA_KEY;
            WebcamName webcamName = opMode.hardwareMap.get(WebcamName.class, "SAMPLING_CAMERA");
            params.cameraName = webcamName;

            camera = new VuforiaCamera(params);
            Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

            pipeline = new SamplingPipeline();

            pipeline.init(opMode.hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);
            pipeline.enable();

            camera.setPipeline(pipeline);
        } else {
            active = false;
        }
    }

    public void enable() {
        camera.enable();
        camera.start();
        active = true;
    }

    public void disable() {
        active = false;
        camera.disable();
        pipeline.disable();
    }

    public GoldPosition getGoldPosition() { return goldPosition; }

    @Override
    public void update() {
        try {
            if (active) {
                if (pipeline.getGoldPoint().x == 0) {
                    goldPosition = GoldPosition.LEFT;
                } else if (pipeline.getGoldPoint().x < CENTER_THRESHOLD) {
                    goldPosition = GoldPosition.CENTER;
                } else {
                    goldPosition = GoldPosition.RIGHT;
                }
            }
        } catch(Exception e) {
            goldPosition = GoldPosition.ERROR;
        }
    }
}
