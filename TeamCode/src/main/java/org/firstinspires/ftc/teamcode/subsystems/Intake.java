package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc12835.library.hardware.management.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake implements Subsystem {

    private DcMotor extenderMotor;
    private DcMotor intakeMotor;
    public Servo pivotServoLeft;
    public Servo pivotServoRight;

    private double extenderPower;
    private double intakePower;
    private double leftPosition;
    private double rightPosition;

    private OpMode opMode;

    public Intake(OpMode opMode) {
        this.opMode = opMode;

        extenderMotor = opMode.hardwareMap.get(DcMotor.class, "EXTENDER");
        intakeMotor = opMode.hardwareMap.get(DcMotor.class, "INTAKE");
        pivotServoLeft = opMode.hardwareMap.get(Servo.class, "PIVOT_LEFT");
        pivotServoRight = opMode.hardwareMap.get(Servo.class, "PIVOT_RIGHT");
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