package org.firstinspires.ftc.teamcode.subsystems;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.ftc12835.library.hardware.management.Subsystem;
import com.ftc12835.library.util.TelemetryUtil;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

public class LatchingLift implements Subsystem {
    // TODO tune these
    public static double LIFT_UP_POSITION = 0;
    public static double LIFT_STOW_POSITION = 0;
    public static double SETPOINT_THRESHOLD = 100;

    private DcMotor liftMotor;
    private DigitalChannel lowerLimitSwitch;
    private DigitalChannel upperLimitSwitch;

    private double liftPower;
    private int setpoint;
    private int liftPosition;

    private TelemetryData telemetryData;

    private Mode mode = Mode.OPEN_LOOP;

    public enum Mode {
        OPEN_LOOP,
        SETPOINT
    }

    private class TelemetryData {
        public int setpoint;
        public int liftPosition;
    }

    public LatchingLift(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotor.class, "LATCHING_LIFT");
        lowerLimitSwitch = hardwareMap.get(DigitalChannel.class, "LOWER_LIMIT");
        upperLimitSwitch = hardwareMap.get(DigitalChannel.class, "UPPER_LIMIT");

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetryData = new TelemetryData();
    }

    public void setLiftPower(double power) {
        liftPower = power;
    }

    public void setSetpoint(int setpoint) { this.setpoint = setpoint; }

    public void resetEncoder() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    @Override
    public Map<String, Object> update(@Nullable Canvas fieldOverlay) {
        liftPosition = liftMotor.getCurrentPosition();
        // override user set power
        if (mode == Mode.SETPOINT) {
            if (liftPower > setpoint - SETPOINT_THRESHOLD && liftPosition < setpoint + SETPOINT_THRESHOLD) {
                liftPower = 0.0;
            } else if (liftPosition < setpoint) {
                liftPower = 1.0;
            } else {
                liftPower = -1.0;
            }
        }

        if (liftPower > 0 && !upperLimitSwitch.getState()) {
            liftMotor.setPower(liftPower);
        } else if (liftPower < 0 && !lowerLimitSwitch.getState()) {
            liftMotor.setPower(liftPower);
        } else {
            // not safe to run
            liftMotor.setPower(0);
        }

        telemetryData.setpoint = setpoint;
        telemetryData.liftPosition = liftPosition;

        return TelemetryUtil.objectToMap(telemetryData);
    }
}
