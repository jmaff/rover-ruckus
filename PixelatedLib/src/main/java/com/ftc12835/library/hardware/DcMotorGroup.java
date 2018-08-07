package com.ftc12835.library.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

/**
 * Allows [DcMotors][DcMotor] to be linked together
 */
public class DcMotorGroup implements DcMotor {
    DcMotor[] motors;

    public DcMotorGroup(DcMotor... motors) {
        this.motors = motors;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return motors[0].getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        for (DcMotor motor : motors) {
            motor.setMotorType(motorType);
        }
    }

    @Override
    public DcMotorController getController() {
        return motors[0].getController();
    }

    @Override
    public int getPortNumber() {
        return motors[0].getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior( zeroPowerBehavior);
        }
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motors[0].getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        for (DcMotor motor : motors) {
            motor.setPowerFloat();
        }
    }

    @Override
    public boolean getPowerFloat() {
        return motors[0].getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        for (DcMotor motor : motors) {
            motor.setTargetPosition(position);
        }
    }

    public void setTargetPositions(int... positions) {
        if (positions.length != motors.length) {
            throw new IllegalArgumentException("The list of target positions is not the same size as the" +
                    " list of motors");
        }

        for (int i = 0; i < positions.length; i++) {
            motors[i].setTargetPosition(positions[i]);
        }
    }

    @Override
    public int getTargetPosition() {
        return motors[0].getTargetPosition();
    }

    public int[] getTargetPositions() {
        int[] positions = new int[motors.length];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = motors[i].getTargetPosition();
        }
        return positions;
    }

    @Override
    public boolean isBusy() {
        boolean isBusy = false;
        for (DcMotor motor : motors) {
            isBusy = isBusy || motor.isBusy();
        }
        return isBusy;
    }

    public enum CurrentPositionMode {
        AVERAGE,
        ADD,
        FIRST_MOTOR
    }

    /**
     * This is a variable that can be set that contains what [CurrentPositionMode] is currently
     * being used. This defaults to [CurrentPositionMode.AVERAGE]
     */
    CurrentPositionMode currentPositionMode = CurrentPositionMode.AVERAGE;

    @Override
    public int getCurrentPosition() {
        int values = 0;
        switch (currentPositionMode) {
            default:
            case AVERAGE:
                for (DcMotor motor : motors) {
                    values += motor.getCurrentPosition();
                }
                return values / motors.length;
            case ADD:
                for (DcMotor motor : motors) {
                    values += motor.getCurrentPosition();
                }
                return values;
            case FIRST_MOTOR:
                return motors[0].getCurrentPosition();
        }
    }

    public void setCurrentPositionMode(CurrentPositionMode positionMode) {
        currentPositionMode = positionMode;
    }

    @Override
    public void setMode(RunMode mode) {
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    @Override
    public RunMode getMode() {
        return motors[0].getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        for (DcMotor motor : motors) {
            motor.setDirection(direction);
        }
    }

    @Override
    public Direction getDirection() {
        return motors[0].getDirection();
    }

    @Override
    public void setPower(double power) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    @Override
    public double getPower() {
        return motors[0].getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return motors[0].getManufacturer();
    }

    @Override
    public String getDeviceName() {
        StringBuilder sb = new StringBuilder();
        for (DcMotor motor : motors) {
            sb.append(motor.getDeviceName());
        }
        return sb.toString();
    }

    @Override
    public String getConnectionInfo() {
        StringBuilder sb = new StringBuilder();
        for (DcMotor motor : motors) {
            sb.append(motor.getConnectionInfo());
        }
        return sb.toString();
    }

    @Override
    public int getVersion() {
        return motors[0].getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        for (DcMotor motor : motors) {
            motor.resetDeviceConfigurationForOpMode();
        }
    }

    @Override
    public void close() {
        for (DcMotor motor : motors) {
            motor.close();
        }
    }
}
