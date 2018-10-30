package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.RobotDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.ftc12835.library.hardware.management.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ThreadPool;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;

public class Robot {
    public static final String TAG = "Robot";
    private List<Subsystem> subsystems;
    private ExecutorService subsystemUpdateExecutor, telemetryUpdateExecutor;

    private boolean enabled;
    private BlockingQueue<TelemetryPacket> telemetryPacketQueue;
    private RobotDashboard dashboard;

    private Runnable subsystemUpdateRunnable = () -> {
        TelemetryPacket telemetryPacket = new TelemetryPacket();

        for (Subsystem subsystem : subsystems) {
            if (subsystem == null) continue;
            try {
                Map<String, Object> telemetryData = subsystem.update(telemetryPacket.fieldOverlay());
                telemetryPacket.putAll(telemetryData);
            } catch (Throwable t) {
                Log.w(TAG, "Subsystem update failed for " + subsystem.toString());
                Log.w(TAG, t);
            }
        }

        telemetryPacketQueue.add(telemetryPacket);
    };

    private Runnable telemetryUpdateRunnable = () -> {
        try {
            TelemetryPacket packet = telemetryPacketQueue.take();
            dashboard.sendTelemetryPacket(packet);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    };

    public MecanumDrive mecanumDrive;
    public LatchingLift latchingLift;
    public Intake intake;

    public Robot(OpMode opMode) {
        dashboard = RobotDashboard.getInstance();
        subsystems = new ArrayList<>();

        subsystemUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem update");
        telemetryUpdateExecutor = ThreadPool.newSingleThreadExecutor("telemetry update");

        telemetryPacketQueue = new ArrayBlockingQueue<>(10);

        mecanumDrive = new MecanumDrive(opMode.hardwareMap);
        latchingLift = new LatchingLift(opMode.hardwareMap);
        intake = new Intake(opMode.hardwareMap);

        subsystems.add(mecanumDrive);
        subsystems.add(latchingLift);
        subsystems.add(intake);
    }

    public void start() {
        if (!enabled) {
            subsystemUpdateExecutor.submit(subsystemUpdateRunnable);
            telemetryUpdateExecutor.submit(telemetryUpdateRunnable);
            enabled = true;
        }
    }

    public void stop() {
        if (subsystemUpdateExecutor != null) {
            subsystemUpdateExecutor.shutdownNow();
            subsystemUpdateExecutor = null;
        }

        if (telemetryUpdateExecutor != null) {
            telemetryUpdateExecutor.shutdownNow();
            telemetryUpdateExecutor = null;
        }
    }
}

