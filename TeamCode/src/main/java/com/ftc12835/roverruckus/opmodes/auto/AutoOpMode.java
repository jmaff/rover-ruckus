package com.ftc12835.roverruckus.opmodes.auto;

import com.acmerobotics.library.vision.VuforiaCamera;
import com.acmerobotics.relicrecovery.configuration.MatchType;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.ftc12835.roverruckus.opmodes.AutoTransitioner;
import com.ftc12835.roverruckus.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AutoOpMode extends LinearOpMode {
    public static final long POLL_INTERVAL = 5; // ms
    public static final double LATERAL_BIAS = 1.25; // in

    protected Robot robot;

    protected VuforiaCamera camera;

    protected abstract void setup();
    protected abstract void run();

    @Override
    public final void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.drive.enablePositionEstimation();
        robot.start();

        camera = new VuforiaCamera();
        camera.initialize();

        String autoTransition = robot.config.getAutoTransition();
        if (!autoTransition.equals(OpModeConfiguration.NO_AUTO_TRANSITION)) {
            AutoTransitioner.transitionOnStop(this, autoTransition);
        }

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
        telemetry.addData("matchType", robot.config.getMatchType());
        if (robot.config.getMatchType() != MatchType.PRACTICE) {
            telemetry.addData("matchNumber", robot.config.getMatchNumber());
        }
        telemetry.addData("delay", robot.config.getDelay());
        telemetry.addData("allianceColor", robot.config.getAllianceColor());
        telemetry.addData("autoTransition", robot.config.getAutoTransition());
        telemetry.update();
    }
}
