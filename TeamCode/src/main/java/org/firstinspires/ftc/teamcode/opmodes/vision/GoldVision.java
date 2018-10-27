package org.firstinspires.ftc.teamcode.opmodes.vision;

import com.ftc12835.library.vision.VisionCamera;
import com.ftc12835.library.vision.VuforiaCamera;
import com.ftc12835.roverruckus.vision.OpenCVGoldPipeline;
import com.ftc12835.roverruckus.vision.SamplingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.VisionConstants;
import org.opencv.core.KeyPoint;


@TeleOp(name = "GoldVision", group = "VISION")
public class GoldVision extends OpMode {
    private VuforiaCamera camera;
    private OpenCVGoldPipeline pipeline;

    private static final String VUFORIA_KEY = "AfqJicT/////AAABmV0Cv8mz10n/joVc93wk6uxXh5eJuApDAXck9Oc1MWCIiRkjrE47UP6ODjidREdy5b3zc1HGJT3/B82inSlLRCa4WpjzbZvHl5FOslDg1EfWtHJH7BhXW8hVVXZJvSSwwmEP8jRWXFudvIMF5QxvVUxcj60RaL1amo00qbROMK5OOJrfDIj4Knbox6pJX17d0WDwJ66XqPbtCQstlPyHSr/iGH81fxLa0cr2Gvk5YiWHNUI4drOz7yPvFzR5Ja7J+MUagIbqtCWmWeUjgEcKqIZw0C4rfJbBsVAJ+Zx58q96hng3GgbXqZLJMcnK1kqOgNXm874Mm+XUQLMWsIIhwBMoMzg0hp5jlDxI0+2XDYGL";

    @Override
    public void init() {
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();
        params.vuforiaLicenseKey = VUFORIA_KEY;
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "SAMPLING_CAMERA");
        params.cameraName = webcamName;

        camera = new VuforiaCamera(params);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        pipeline = new OpenCVGoldPipeline();

        pipeline.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, true);
        pipeline.enable();

        camera.setDogeCVDetector(pipeline);

        camera.enableDogeCV();
        camera.enableTrack();
        camera.start();
    }

    @Override
    public void loop() {
        telemetry.addData("x", pipeline.getGoldPoint().x);
        telemetry.addData("y", pipeline.getGoldPoint().y);
    }

    @Override
    public void stop() {
        camera.stop();
    }
}
