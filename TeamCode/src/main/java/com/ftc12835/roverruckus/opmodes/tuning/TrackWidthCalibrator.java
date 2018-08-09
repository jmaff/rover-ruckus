package com.ftc12835.roverruckus.opmodes.tuning;

import com.acmerobotics.library.hardware.LynxOptimizedI2cFactory;
import com.acmerobotics.roadrunner.drive.Drive;
import com.ftc12835.roverruckus.roadrunner.TrackWidthCalibrationOpMode;
import com.ftc12835.roverruckus.subsystems.MecanumDrive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

public class TrackWidthCalibrator extends TrackWidthCalibrationOpMode {
    @Override
    protected Drive initDrive() {
        return new MecanumDrive(hardwareMap);
    }

    @Override
    protected BNO055IMU initIMU() {
        LynxModule frontHub = hardwareMap.get(LynxModule.class, "frontHub");
        I2cDeviceSynch imuI2cDevice = LynxOptimizedI2cFactory.createLynxI2cDeviceSynch(frontHub, 0);
        imuI2cDevice.setUserConfiguredName("imu");
        BNO055IMU imu = new LynxEmbeddedIMU(imuI2cDevice);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        return imu;
    }
}
