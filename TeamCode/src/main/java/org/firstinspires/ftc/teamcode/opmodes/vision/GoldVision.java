package org.firstinspires.ftc.teamcode.opmodes.vision;

import com.ftc12835.library.vision.VisionCamera;
import com.ftc12835.roverruckus.vision.SamplingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.VisionConstants;
import org.opencv.core.KeyPoint;

@TeleOp(name = "GoldVision", group = "VISION")
public class GoldVision extends OpMode {
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
        telemetry.addData("x", samplingPipeline.getGoldPoint().x);
        telemetry.addData("y", samplingPipeline.getGoldPoint().y);

        String position;
        switch (samplingPipeline.getGoldPosition()) {
            case LEFT:
                position = "LEFT";
            case CENTER:
                position = "CENTER";
            case RIGHT:
                position = "RIGHT";
            default:
            case UNKNOWN:
                position = "UNKNOWN";
        }

        telemetry.addData("Gold Position", position);
    }

    @Override
    public void stop() {
        camera.close();
    }
}
