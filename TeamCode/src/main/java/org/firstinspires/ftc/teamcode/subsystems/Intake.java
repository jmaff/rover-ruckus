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

    private DcMotor extenderMotor;
    private DcMotor intakeMotor;
    public Servo pivotServoLeft;
    public Servo pivotServoRight;

    private double extenderPower;
    private double intakePower;
    private double leftPosition;
    private double rightPosition;

    public Intake(HardwareMap hardwareMap) {
        extenderMotor = hardwareMap.get(DcMotor.class, "EXTENDER");
        intakeMotor = hardwareMap.get(DcMotor.class, "INTAKE");
        pivotServoLeft = hardwareMap.get(Servo.class, "PIVOT_LEFT");
        pivotServoRight = hardwareMap.get(Servo.class, "PIVOT_RIGHT");
    }

    public void setExtenderPower(double extenderPower) {
        this.extenderPower = extenderPower;
    }

    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
    }

    public void setLeftPosition(double leftPosition) {
        this.leftPosition = leftPosition;
    }

    public void setRightPosition(double rightPosition) {
        this.rightPosition = rightPosition;
    }

    @Override
    public void update() {
        extenderMotor.setPower(extenderPower);
        intakeMotor.setPower(intakePower);

        pivotServoLeft.setPosition(leftPosition);
        pivotServoRight.setPosition(rightPosition);
    }
}