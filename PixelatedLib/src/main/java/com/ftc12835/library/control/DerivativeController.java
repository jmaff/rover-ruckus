package com.ftc12835.library.control;

/**
 * Created by 21maffetone on 8/11/18.
 */

public class DerivativeController {
    private final double kD;
    private double setpoint;
    private double previousError = 0;

    public DerivativeController(double kD) {
        this.kD = kD;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getError(double actual) {
        return actual - setpoint;
    }

    public double update(double error) {
        double deltaError = error - previousError;
        previousError = error;
        return kD * deltaError;
    }
}
