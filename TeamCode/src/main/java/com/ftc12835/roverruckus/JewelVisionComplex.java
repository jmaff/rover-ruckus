package com.ftc12835.roverruckus;

import com.acmerobotics.library.vision.FpsTracker;
import com.acmerobotics.library.vision.OpenCVCamera;
import com.acmerobotics.library.vision.VisionCamera;
import com.acmerobotics.relicrecovery.vision.FixedJewelTracker;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.android.OpenCVLoader;

@TeleOp(name = "JewelVisionComplex", group = "VISION")
public class JewelVisionComplex extends OpMode {
    private VisionCamera camera;
    private FixedJewelTracker jewelTracker;

    @Override
    public void init() {
        jewelTracker = new FixedJewelTracker();
        camera = new OpenCVCamera();
        camera.addTracker(jewelTracker);
        camera.addTracker(new FpsTracker());
        camera.initialize();
    }

    @Override
    public void loop() {
        telemetry.addData("red", "%.2f / %.2f", jewelTracker.getLeftRed(), jewelTracker.getRightRed());
        telemetry.addData("blue", "%.2f / %.2f", jewelTracker.getLeftBlue(), jewelTracker.getRightBlue());
    }
}
