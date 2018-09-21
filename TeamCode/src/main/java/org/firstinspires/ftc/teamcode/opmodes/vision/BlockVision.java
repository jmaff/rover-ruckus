package org.firstinspires.ftc.teamcode.opmodes.vision;

import com.ftc12835.library.vision.VisionCamera;
import com.ftc12835.roverruckus.vision.SamplingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.VisionConstants;
import org.opencv.core.KeyPoint;

@TeleOp(name = "BlockVision", group = "VISION")
public class BlockVision extends OpMode {
    private VisionCamera camera;
    private SamplingPipeline samplingPipeline;

    @Override
    public void init() {
        samplingPipeline = new SamplingPipeline();
        camera = new VisionCamera();
        camera.addPipeline(samplingPipeline);
        camera.init();
    }

    @Override
    public void loop() {
        if (samplingPipeline.getBlobs().toArray().length > 0) {
            telemetry.addData("x", samplingPipeline.getBlobs().toArray()[0].pt.x);
            telemetry.addData("x", samplingPipeline.getBlobs().toArray()[0].pt.y);
        }
    }

    @Override
    public void stop() {
        camera.close();
    }
}
