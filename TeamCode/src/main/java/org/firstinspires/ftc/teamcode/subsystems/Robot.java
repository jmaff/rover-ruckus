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
    private ExecutorService subsystemUpdateExecutor;

    private boolean enabled;

    public MecanumDrive mecanumDrive;
    public LatchingLift latchingLift;
    public Intake intake;
    public Outtake outtake;

    public Robot(OpMode opMode) {
        subsystems = new ArrayList<>();

        subsystemUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem update");

        mecanumDrive = new MecanumDrive(opMode.hardwareMap);
        latchingLift = new LatchingLift(opMode.hardwareMap);
        intake = new Intake(opMode.hardwareMap);
        outtake = new Outtake(opMode.hardwareMap);

        subsystems.add(mecanumDrive);
        subsystems.add(latchingLift);
        subsystems.add(intake);
        subsystems.add(outtake);
    }

    public void update() {
        if (enabled) {
            for (Subsystem subsystem : subsystems) {
                if (subsystem == null) continue;
                try {
                    subsystem.update();
                } catch (Throwable t) {
                    Log.w(TAG, "Subsystem update failed for " + subsystem.toString());
                    Log.w(TAG, t);
                }
            }
        }
    }

    public void start() {
        enabled = true;
    }

    public void stop() {
        if (subsystemUpdateExecutor != null) {
            subsystemUpdateExecutor.shutdownNow();
            subsystemUpdateExecutor = null;
        }
    }


}

