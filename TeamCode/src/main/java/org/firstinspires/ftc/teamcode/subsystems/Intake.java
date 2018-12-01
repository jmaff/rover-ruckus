package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.ftc12835.library.hardware.management.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class Intake implements Subsystem {

    private DcMotor extenderMotor;
    private DcMotor intakeMotor;
    public CRServo pivotServoLeft;
    public CRServo pivotServoRight;

    private double extenderPower;
    private double intakePower;
    private double leftPosition;
    private double rightPosition;

    private OpMode opMode;
    private PivotPosition currentPivotPosition = PivotPosition.UP;

    public static double LEFT_STOW = 1.00;
    public static double LEFT_DEPLOY= 0.00;
    public static double RIGHT_STOW = 0.00;
    public static double RIGHT_DEPLOY = 1.00;

    public static double LEFT_MIDDLE = 0.30;
    public static double RIGHT_MIDDLE = 0.70;

    public enum PivotPosition {
        UP,
        OFF,
        DOWN
    }

    public Intake(OpMode opMode) {
        this.opMode = opMode;

        extenderMotor = opMode.hardwareMap.get(DcMotor.class, "EXTENDER");
        intakeMotor = opMode.hardwareMap.get(DcMotor.class, "INTAKE");
        pivotServoLeft = opMode.hardwareMap.get(CRServo.class, "PIVOT_LEFT");
        pivotServoRight = opMode.hardwareMap.get(CRServo.class, "PIVOT_RIGHT");

        setIntakePivotPosition(PivotPosition.UP);
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

    public PivotPosition getCurrentPivotPosition() {
        return currentPivotPosition;
    }

    public void setIntakePivotPosition(PivotPosition pivotPosition) {
        currentPivotPosition = pivotPosition;
        switch (pivotPosition) {
            case UP:
                setLeftPosition(1.00);
                setRightPosition(-1.00);
                break;
            case OFF:
                setLeftPosition(0);
                setRightPosition(0);
                break;
            case DOWN:
                setLeftPosition(-1.00);
                setRightPosition(1.00);
                break;
        }
    }

    public void dumpMarker() {
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        setIntakePivotPosition(PivotPosition.DOWN);
        linearOpMode.sleep(900);
        setIntakePivotPosition(PivotPosition.UP);
        linearOpMode.sleep(900);
        setIntakePivotPosition(PivotPosition.OFF);
    }


    @Override
    public void update() {
        extenderMotor.setPower(extenderPower);
        intakeMotor.setPower(intakePower);

        pivotServoLeft.setPower(leftPosition);
        pivotServoRight.setPower(rightPosition);
    }
}