package com.ftc12835.library.control;

public class IntegralController {
    private final double kI;
    private double setpoint;

    public IntegralController(double kI) {
        this.kI = kI;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getError(double actual) {
        return actual - setpoint;
    }

    // TODO: implement I control
    public double update(double error) {
        return 0.0;
    }
}
