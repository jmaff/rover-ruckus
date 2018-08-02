package com.ftc12835.roverruckus.opmodes.vision;

import com.acmerobotics.library.vision.VuforiaCamera;
import com.ftc12835.roverruckus.vision.DynamicBlockTracker;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BlockVision", group = "VISION")
public class BlockVision extends OpMode {
    private VuforiaCamera camera;
    private DynamicBlockTracker blockTracker;

    @Override
    public void init() {
        blockTracker = new DynamicBlockTracker();
        camera = new VuforiaCamera();
        camera.addTracker(blockTracker);
        camera.initialize();
    }

    @Override
    public void loop() {
        telemetry.addData("Blocks Detected", blockTracker.getBlocksDetected());
    }
}
