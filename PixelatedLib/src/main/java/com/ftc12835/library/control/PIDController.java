package com.ftc12835.library.control;

public class PIDController {
    private final double kP;
    private final double kI;
    private final double kD;

    private double setpoint;

    private double rollingError = 0;
    private double prevError;
    private double deltaError = 0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getError(double actual) {
        return actual - setpoint;
    }

    public double update(double actual) {
        double error = getError(actual);

        double pOutput = kP * error;

        rollingError += error;
        double iOutput = kI * rollingError;

        deltaError = error - prevError;
        prevError = error;
        double dOutput = kD * deltaError;

        return pOutput + iOutput + dOutput;
    }

    public boolean isDone() {
        return Math.abs(deltaError) < 0.1;
    }
}
