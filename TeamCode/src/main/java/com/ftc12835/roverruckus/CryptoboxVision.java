package com.ftc12835.roverruckus;

import com.acmerobotics.library.vision.FpsTracker;
import com.acmerobotics.library.vision.VisionCamera;
import com.acmerobotics.library.vision.VuforiaCamera;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by 21maffetone on 7/18/18.
 */

@TeleOp(name = "CryptoboxVision", group = "VISION")
public class CryptoboxVision extends OpMode {
        private VisionCamera camera;
        private CryptoboxTracker cryptoboxTracker;
        private FpsTracker fpsTracker;

        @Override
        public void init() {
            camera = new VuforiaCamera();
            cryptoboxTracker = new CryptoboxTracker(AllianceColor.BLUE);
            fpsTracker = new FpsTracker();
            camera.addTracker(cryptoboxTracker);
            camera.addTracker(fpsTracker);
            camera.initialize();
        }

        @Override
        public void loop() {
        }
}
