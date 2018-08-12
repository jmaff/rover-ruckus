package com.ftc12835.library.hardware.devices;

import android.support.annotation.NonNull;

import com.acmerobotics.library.hardware.LynxOptimizedI2cFactory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class IMU implements BNO055IMU {
    private BNO055IMU delegate;
    public enum HubOrientation {
        HORIZONTAL,
        VERTICAL
    }

    public IMU(REVHub hub, HubOrientation orientation, int port, String name) {
        I2cDeviceSynch imuI2cDevice = LynxOptimizedI2cFactory.createLynxI2cDeviceSynch(hub.getDelegate(), port);
        imuI2cDevice.setUserConfiguredName(name);
        delegate = new LynxEmbeddedIMU(imuI2cDevice);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        delegate.initialize(parameters);

        if (orientation == HubOrientation.VERTICAL) {
            // taken from https://ftcforum.usfirst.org/forum/ftc-technology/53812-mounting-the-revhub-vertically?p=56587#post56587
            // testing suggests that an axis remap is more accurate then simply changing the axis read
            // we hypothesize that this helps properly configure the internal sensor fusion/Kalman filtering
            try {
                // axis remap
                byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
                byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

                // Need to be in CONFIG mode to write to registers
                write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

                Thread.sleep(100); //Changing modes requires a delay before doing anything else

                // Write to the AXIS_MAP_CONFIG register
                write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);

                // Write to the AXIS_MAP_SIGN register
                write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);

                // Need to change back into the IMU mode to use the gyro
                write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);

                Thread.sleep(100); //Changing modes again requires a delay
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public IMU(REVHub hub, HubOrientation orientation, int port) {
        this(hub, orientation, port, "imu");
    }

    public IMU(REVHub hub, HubOrientation orientation) {
        this(hub, orientation, 0);
    }

    public IMU(REVHub hub) {
        this(hub, HubOrientation.HORIZONTAL);
    }


    @Override
    public boolean initialize(@NonNull Parameters parameters) {
        return delegate.initialize(parameters);
    }

    @NonNull
    @Override
    public Parameters getParameters() {
        return delegate.getParameters();
    }

    @Override
    public void close() {
        delegate.close();
    }

    @Override
    public Orientation getAngularOrientation() {
        return delegate.getAngularOrientation();
    }

    @Override
    public Orientation getAngularOrientation(AxesReference reference, AxesOrder order, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit) {
        return delegate.getAngularOrientation(reference, order, angleUnit);
    }

    @Override
    public Acceleration getOverallAcceleration() {
        return delegate.getOverallAcceleration();
    }

    @Override
    public AngularVelocity getAngularVelocity() {
        return delegate.getAngularVelocity();
    }

    @Override
    public Acceleration getLinearAcceleration() {
        return delegate.getLinearAcceleration();
    }

    @Override
    public Acceleration getGravity() {
        return delegate.getGravity();
    }

    @Override
    public Temperature getTemperature() {
        return delegate.getTemperature();
    }

    @Override
    public MagneticFlux getMagneticFieldStrength() {
        return delegate.getMagneticFieldStrength();
    }

    @Override
    public Quaternion getQuaternionOrientation() {
        return delegate.getQuaternionOrientation();
    }

    @Override
    public Position getPosition() {
        return delegate.getPosition();
    }

    @Override
    public Velocity getVelocity() {
        return delegate.getVelocity();
    }

    @Override
    public Acceleration getAcceleration() {
        return delegate.getAcceleration();
    }

    @Override
    public void startAccelerationIntegration(Position initialPosition, Velocity initialVelocity, int msPollInterval) {
        delegate.startAccelerationIntegration(initialPosition, initialVelocity, msPollInterval);
    }

    @Override
    public void stopAccelerationIntegration() {
        delegate.stopAccelerationIntegration();
    }

    @Override
    public SystemStatus getSystemStatus() {
        return delegate.getSystemStatus();
    }

    @Override
    public SystemError getSystemError() {
        return delegate.getSystemError();
    }

    @Override
    public CalibrationStatus getCalibrationStatus() {
        return delegate.getCalibrationStatus();
    }

    @Override
    public boolean isSystemCalibrated() {
        return delegate.isSystemCalibrated();
    }

    @Override
    public boolean isGyroCalibrated() {
        return delegate.isGyroCalibrated();
    }

    @Override
    public boolean isAccelerometerCalibrated() {
        return delegate.isAccelerometerCalibrated();
    }

    @Override
    public boolean isMagnetometerCalibrated() {
        return delegate.isMagnetometerCalibrated();
    }

    @Override
    public CalibrationData readCalibrationData() {
        return delegate.readCalibrationData();
    }

    @Override
    public void writeCalibrationData(CalibrationData data) {
        delegate.writeCalibrationData(data);
    }

    @Override
    public byte read8(Register register) {
        return delegate.read8(register);
    }

    @Override
    public byte[] read(Register register, int cb) {
        return delegate.read(register, cb);
    }

    @Override
    public void write8(Register register, int bVal) {
        delegate.write8(register, bVal);
    }

    @Override
    public void write(Register register, byte[] data) {
        delegate.write(register, data);
    }
}
