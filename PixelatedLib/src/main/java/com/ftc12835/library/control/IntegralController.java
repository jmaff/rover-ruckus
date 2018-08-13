package com.ftc12835.library.control;

public class IntegralController {
    private final double kI;
    private double setpoint;
    private double sum = 0;

    public IntegralController(double kI) {
        this.kI = kI;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getError(double actual) {
        return actual - setpoint;
    }

    public double update(double error) {
        sum += error;
        return kI * sum;
    }
}
