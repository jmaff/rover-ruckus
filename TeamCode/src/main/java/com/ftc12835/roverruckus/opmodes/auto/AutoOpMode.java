package com.ftc12835.roverruckus.opmodes.auto;

import com.acmerobotics.library.vision.VuforiaCamera;
import com.acmerobotics.relicrecovery.configuration.MatchType;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.ftc12835.roverruckus.opmodes.AutoTransitioner;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AutoOpMode extends LinearOpMode {
    public static final long POLL_INTERVAL = 5; // ms
    public static final double LATERAL_BIAS = 1.25; // in

    protected VuforiaCamera camera;

    protected abstract void setup();
    protected abstract void run();

    @Override
    public final void runOpMode() throws InterruptedException {

        camera = new VuforiaCamera();
        camera.initialize();


        setup();

        displayInitTelemetry();

        waitForStart();

        telemetry.clearAll();
        telemetry.update();


        if (isStopRequested()) {
            return;
        }

        run();
    }

    private void displayInitTelemetry() {
        telemetry.update();
    }
}
