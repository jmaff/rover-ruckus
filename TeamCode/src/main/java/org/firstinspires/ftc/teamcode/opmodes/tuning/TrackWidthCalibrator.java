package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.roadrunner.TrackWidthCalibrationOpMode;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class TrackWidthCalibrator extends TrackWidthCalibrationOpMode {
    @Override
    protected Drive initDrive() {
        return null;
    }

    // TODO fix this
    @Override
    protected BNO055IMU initIMU() {
        return null;
    }
}
