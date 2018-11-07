package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.roadrunner.drive.Drive;

import org.firstinspires.ftc.teamcode.roadrunner.FeedforwardTuningOpMode;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class FeedforwardTuner extends FeedforwardTuningOpMode {
    public FeedforwardTuner() {
        super(100.0, MecanumDrive.MOTOR_CONFIG.getMaxRPM(), 4.0);
    }

    @Override
    protected Drive initDrive() {
        return null;
    }
}
