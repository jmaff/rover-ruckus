package com.ftc12835.roverruckus.opmodes.tuning;

import com.acmerobotics.roadrunner.drive.Drive;
import com.ftc12835.roverruckus.roadrunner.FeedforwardTuningOpMode;
import com.ftc12835.roverruckus.subsystems.MecanumDrive;

public class FeedforwardTuner extends FeedforwardTuningOpMode {
    public FeedforwardTuner() {
        super(100.0, MecanumDrive.MOTOR_CONFIG.getMaxRPM(), 4.0);
    }

    @Override
    protected Drive initDrive() {
        return new MecanumDrive(hardwareMap);
    }
}
