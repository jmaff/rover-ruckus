package com.ftc12835.library.control;

public class ProportionalController {
    private final double kP;
    private double setpoint;

    public ProportionalController(double kP) {
        this.kP = kP;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getError(double actual) {
        return actual - setpoint;
    }

    public double update(double error) {
        return kP * error;
    }
}
