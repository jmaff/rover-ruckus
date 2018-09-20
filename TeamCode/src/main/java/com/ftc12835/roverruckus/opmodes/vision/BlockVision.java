package com.ftc12835.roverruckus.opmodes.vision;

import com.ftc12835.roverruckus.vision.SamplingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

@TeleOp(name = "BlockVision", group = "VISION")
public class BlockVision extends OpMode {
    private SamplingPipeline samplingPipeline;

    @Override
    public void init() {
        samplingPipeline = new SamplingPipeline();
        samplingPipeline.init(hardwareMap, CameraViewDisplay.getInstance());
        samplingPipeline.enable();
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        samplingPipeline.disable();
    }
}
