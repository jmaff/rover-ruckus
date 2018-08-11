package com.ftc12835.library.control;

/**
 * Created by 21maffetone on 8/11/18.
 */

public class DerivativeController {
    private final double kD;
    private double setpoint;

    public DerivativeController(double kD) {
        this.kD = kD;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getError(double actual) {
        return actual - setpoint;
    }

    // TODO: implement D control
    public double update(double error) {
        return 0.0;
    }
}
