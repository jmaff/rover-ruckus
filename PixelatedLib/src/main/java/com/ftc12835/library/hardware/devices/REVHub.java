package com.ftc12835.library.hardware.devices;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.LynxUsbDevice;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.exception.RobotCoreException;

public class REVHub {
    private LynxModule delegate;
    private LynxGetBulkInputDataResponse response;

    public REVHub(LynxModule delegate) {
        this.delegate = delegate;

    }

    public void pull() {
        LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(delegate);
        try {
            response = command.sendReceive();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (LynxNackException e) {
            // TODO: no idea what we need to do here
        }
    }

    public void enablePhoneCharging(boolean value) {
        try {
            delegate.enablePhoneCharging(false);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (LynxNackException e) {
            // TODO: no idea what we need to do here
        } catch (RobotCoreException e) {
            Log.w("REV Hub", e);
        }
    }

    public LynxModule getDelegate() {
        return delegate;
    }

    public int getEncoder(int motorZ) {
        return (response != null) ? response.getEncoder(motorZ) : 0;
    }

    public boolean getDigitalInput(int digitalInputZ) {
        return (response != null) ? response.getDigitalInput(digitalInputZ) : false;
    }

    public int getAnalogInput(int inputZ) {
        return (response != null) ? response.getAnalogInput(inputZ) : 0;
    }

    public int getVelocity(int motorZ) {
        return (response != null) ? response.getVelocity(motorZ) : 0;
    }

    public boolean isAtTarget(int motorZ) {
        return (response != null) ? response.isAtTarget(motorZ) : false;
    }

    public boolean isOverCurrent(int motorZ) {
        return (response != null) ? response.isOverCurrent(motorZ) : false;
    }
}
