package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.ftc12835.library.hardware.management.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    public static final String TAG = "Robot";
    private List<Subsystem> subsystems;

    private boolean enabled;

    public MecanumDrive mecanumDrive;
    public LatchingLift latchingLift;
    public Intake intake;
    public Outtake outtake;

    public Robot(OpMode opMode) {
        subsystems = new ArrayList<>();

        mecanumDrive = new MecanumDrive(opMode);
        latchingLift = new LatchingLift(opMode);
        intake = new Intake(opMode);
        outtake = new Outtake(opMode);

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
        enabled = false;
    }


}

