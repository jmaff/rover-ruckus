package com.ftc12835.roverruckus.opmodes.vision;

import com.acmerobotics.library.vision.FpsTracker;
import com.acmerobotics.library.vision.OpenCVCamera;
import com.acmerobotics.library.vision.VisionCamera;
import com.acmerobotics.library.vision.VuforiaCamera;
import com.acmerobotics.relicrecovery.vision.DynamicJewelTracker;
import com.acmerobotics.relicrecovery.vision.FixedJewelTracker;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.android.OpenCVLoader;

@TeleOp(name = "JewelVisionComplex", group = "VISION")
public class JewelVisionComplex extends OpMode {
    private VuforiaCamera camera;
    private DynamicJewelTracker jewelTracker;

    @Override
    public void init() {
        jewelTracker = new DynamicJewelTracker();
        camera = new VuforiaCamera();
        camera.addTracker(jewelTracker);
        camera.initialize();
    }

    @Override
    public void loop() {
        telemetry.addData("jewel", jewelTracker.getJewelPosition());
    }
}
