package org.firstinspires.ftc.teamcode.subsystems;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.ftc12835.library.hardware.management.Subsystem;
import com.ftc12835.library.util.TelemetryUtil;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Map;

public class Intake implements Subsystem {
    // TODO tune these
    public static final int EXTENDER_STOW_POSITION = 0;
    public static final int EXTENDER_READY_POSITION = 0;
    public static double SETPOINT_THRESHOLD = 100;

    public static double LEFT_PIVOT_STOW_FULL = 0.0;
    public static final double LEFT_PIVOT_STOW_IDLE = 0.43;
    public static final double LEFT_PIVOT_DEPLOY_FULL = 1.0;
    public static final double LEFT_PIVOT_DEPLOY_IDLE = 0.48;
    public static final double LEFT_PIVOT_RAISED = 0.5;

    public static double RIGHT_PIVOT_STOW_FULL = 1.0;
    public static final double RIGHT_PIVOT_STOW_IDLE = 0.48;
    public static final double RIGHT_PIVOT_DEPLOY_FULL = 0.0;
    public static final double RIGHT_PIVOT_DEPLOY_IDLE = 0.43;
    public static final double RIGHT_PIVOT_RAISED = 0.5;

    public static final long TIME_TO_DEPLOY = 1000;
    public static final long TIME_TO_STOW = 1000;
    public static final long TIME_TO_RAISED = 1000;

    private DcMotor extenderMotor;
    private DcMotor intakeMotor;
    private Servo pivotServoLeft;
    private Servo pivotServoRight;
    private DigitalChannel innerLimitSwitch;

    private double extenderPower;
    private int extenderPosition;
    private int extenderSetpoint;
    private double intakePower;
    private long startPivotTime;

    private TelemetryData telemetryData;

    private class TelemetryData {
        public int extenderSetpoint;
        public int extenderPosition;
    }

    private Mode mode = Mode.OPEN_LOOP;

    public enum Mode {
        OPEN_LOOP,
        SETPOINT
    }

    private IntakePosition intakePosition = IntakePosition.STOW;
    private IntakePosition intakeSetpoint = IntakePosition.STOW;

    private boolean deploying;

    public enum IntakePosition {
        STOW,
        DEPLOY,
        RAISED
    }

    public Intake(HardwareMap hardwareMap) {
        extenderMotor = hardwareMap.get(DcMotor.class, "EXTENDER");
        intakeMotor = hardwareMap.get(DcMotor.class, "INTAKE");
        pivotServoLeft = hardwareMap.get(Servo.class, "PIVOT_LEFT");
        pivotServoRight = hardwareMap.get(Servo.class, "PIVOT_RIGHT");
        innerLimitSwitch = hardwareMap.get(DigitalChannel.class, "EXTENDER_LIMIT");

        telemetryData = new TelemetryData();

    }

    public void setExtenderPower(double extenderPower) {
        this.extenderPower = extenderPower;
    }

    public void setExtenderPosition(int extenderPosition) {
        this.extenderPosition = extenderPosition;
    }

    public void setExtenderSetpoint(int extenderSetpoint) {
        this.extenderSetpoint = extenderSetpoint;
    }

    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    public void setIntakeSetpoint(IntakePosition intakePosition) {
        this.intakeSetpoint = intakeSetpoint;
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public boolean isExtending() {
        if (mode == Mode.SETPOINT) {
            return extenderPower != 0.0;
        }
        return false;
    }

    public boolean isDeploying() {
        return deploying;
    }

    @Override
    public Map<String, Object> update(@Nullable Canvas fieldOverlay) {
        extenderPosition = extenderMotor.getCurrentPosition();

        // override user set power
        if (mode == Mode.SETPOINT) {
            if (extenderPosition > extenderSetpoint - SETPOINT_THRESHOLD
                    && extenderPosition < extenderPosition + SETPOINT_THRESHOLD) {
                extenderPower = 0.0;
            } else if (extenderPosition < extenderSetpoint) {
                extenderPower = 1.0;
            } else {
                extenderPower = -1.0;
            }
        }

        if (extenderPower < 0 && innerLimitSwitch.getState()) {
            // not safe to run
            extenderMotor.setPower(0);
        } else {
            extenderMotor.setPower(extenderPower);
        }

        // time to move the thing
        if (intakeSetpoint != intakePosition && !deploying) {
            deploying = true;
            startPivotTime = System.currentTimeMillis();

            switch (intakeSetpoint) {
                case STOW:
                    pivotServoLeft.setPosition(LEFT_PIVOT_STOW_FULL);
                    pivotServoRight.setPosition(RIGHT_PIVOT_STOW_FULL);
                    break;
                case DEPLOY:
                    pivotServoLeft.setPosition(LEFT_PIVOT_DEPLOY_FULL);
                    pivotServoRight.setPosition(RIGHT_PIVOT_DEPLOY_FULL);
                    break;
                case RAISED:
                    pivotServoLeft.setPosition(LEFT_PIVOT_STOW_FULL);
                    pivotServoRight.setPosition(RIGHT_PIVOT_STOW_FULL);
                    break;
            }
        } else if (deploying) {
            if (System.currentTimeMillis() - startPivotTime >= TIME_TO_RAISED && intakeSetpoint == IntakePosition.RAISED) {
                pivotServoLeft.setPosition(LEFT_PIVOT_RAISED);
                pivotServoRight.setPosition(RIGHT_PIVOT_RAISED);
                deploying = false;
                intakePosition = intakeSetpoint;
            } else if (System.currentTimeMillis() - startPivotTime >= TIME_TO_STOW && intakeSetpoint == IntakePosition.STOW) {
                pivotServoLeft.setPosition(LEFT_PIVOT_STOW_IDLE);
                pivotServoRight.setPosition(RIGHT_PIVOT_STOW_IDLE);
                deploying = false;
                intakePosition = intakeSetpoint;
            } else if (System.currentTimeMillis() - startPivotTime >= TIME_TO_DEPLOY && intakeSetpoint == IntakePosition.DEPLOY) {
                pivotServoLeft.setPosition(LEFT_PIVOT_DEPLOY_IDLE);
                pivotServoRight.setPosition(RIGHT_PIVOT_DEPLOY_IDLE);
                deploying = false;
                intakePosition = intakeSetpoint;
            }
        }

        intakeMotor.setPower(intakePower);

        telemetryData.extenderPosition = extenderPosition;
        telemetryData.extenderPosition = extenderSetpoint;

        return TelemetryUtil.objectToMap(telemetryData);
    }
}