package com.ftc12835.roverruckus.opmodes.vision;

import com.acmerobotics.library.vision.VuforiaCamera;
import com.ftc12835.roverruckus.subsystems.SamplingCamera;
import com.ftc12835.roverruckus.vision.DynamicBlockTracker;
import com.ftc12835.roverruckus.vision.SamplingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BlockVision", group = "VISION")
public class BlockVision extends OpMode {
    private SamplingCamera samplingCamera;

    @Override
    public void init() {
        samplingCamera = new SamplingCamera(hardwareMap);
        samplingCamera.attachPipeline(new SamplingPipeline());
        samplingCamera.init();
    }

    @Override
    public void loop() {
    }

    @Override
    public void stop() {
        super.stop();
        samplingCamera.close();
    }
}
