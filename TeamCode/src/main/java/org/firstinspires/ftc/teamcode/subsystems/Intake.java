package org.firstinspires.ftc.teamcode.subsystems;

import android.text.method.Touch;

import com.acmerobotics.dashboard.config.Config;
import com.ftc12835.library.hardware.management.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake implements Subsystem {

    private DcMotor extenderMotor;
    private DcMotor intakeMotor;
    public Servo pivotServoLeft;
    public Servo pivotServoRight;

    private DigitalChannel intakeLimit;

    DistanceSensor distance1;
    DistanceSensor distance2;

    private double extenderPower;
    private double intakePower;
    private double leftPosition;
    private double rightPosition;

    private OpMode opMode;
    private PivotPosition currentPivotPosition = PivotPosition.UP;

    public static double LEFT_DOWN = 1.00;
    public static double LEFT_UP= 0.30;
    public static double RIGHT_DOWN = 0.00;
    public static double RIGHT_UP = 0.70;

    public static double LEFT_MIDDLE = 0.70;
    public static double RIGHT_MIDDLE = 0.30;

    public enum PivotPosition {
        UP,
        MIDDLE,
        DOWN
    }

    public enum MineralStatus {
        TWO,
        ONE,
        NONE
    }

    public Intake(OpMode opMode) {
        this.opMode = opMode;

        extenderMotor = opMode.hardwareMap.get(DcMotor.class, "EXTENDER");
        extenderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetExtenderEncoder();

        intakeMotor = opMode.hardwareMap.get(DcMotor.class, "INTAKE");
        pivotServoLeft = opMode.hardwareMap.get(Servo.class, "PIVOT_LEFT");
        pivotServoRight = opMode.hardwareMap.get(Servo.class, "PIVOT_RIGHT");
        intakeLimit = opMode.hardwareMap.get(DigitalChannel.class, "INTAKE_LIMIT");

        distance1 = opMode.hardwareMap.get(DistanceSensor.class, "MINERAL_1");
        distance2 = opMode.hardwareMap.get(DistanceSensor.class, "MINERAL_2");

        setIntakePivotPosition(PivotPosition.UP);
    }

    public void resetExtenderEncoder() {
        extenderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extenderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
                setLeftPosition(LEFT_UP);
                setRightPosition(RIGHT_UP);
                break;
            case MIDDLE:
                setLeftPosition(LEFT_MIDDLE);
                setRightPosition(RIGHT_MIDDLE);
                break;
            case DOWN:
                setLeftPosition(LEFT_DOWN);
                setRightPosition(RIGHT_DOWN);
                break;
        }
    }

    public int getExtenderPosition() {
        return extenderMotor.getCurrentPosition();
    }

    public void dumpMarker() {
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        setIntakePivotPosition(PivotPosition.MIDDLE);
        linearOpMode.sleep(900);
        setIntakePivotPosition(PivotPosition.UP);
    }

    public void runExtenderToPosition(double power, int counts) {
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        setExtenderPower(power);

        while (linearOpMode.opModeIsActive()) {
            if (Math.abs(getExtenderPosition()) > counts) {
                break;
            }
        }

        setExtenderPower(0.0);
    }

    public void retractExtenderToPosition(double power, int counts) {
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        setExtenderPower(power);

        while (linearOpMode.opModeIsActive()) {
            if (Math.abs(getExtenderPosition()) < counts) {
                break;
            }
        }

        setExtenderPower(0.0);
    }

    public void retractIntakeExtender() {
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        setExtenderPower(1.0);

        long start = System.currentTimeMillis();

        while (linearOpMode.opModeIsActive()) {
            if (getIntakeLimit()) {
                break;
            }

            if (System.currentTimeMillis() - start >= 2500) {
                break;
            }
        }

        setExtenderPower(0.0);
    }

    public boolean getIntakeLimit() {
        return !intakeLimit.getState();
    }

    public double getUpperDistance() {
        return distance1.getDistance(DistanceUnit.CM);
    }

    public double getLowerDistance() {
        return distance2.getDistance(DistanceUnit.CM);
    }

    public MineralStatus getMineralStatus() {
        if (getLowerDistance() < 5.7 && getUpperDistance() < 5.7) {
            return MineralStatus.TWO;
        } else if (getLowerDistance() < 5.7 || getUpperDistance() < 5.7) {
            return MineralStatus.ONE;
        } else {
            return MineralStatus.NONE;
        }
    }


    @Override
    public void update() {
        extenderMotor.setPower(extenderPower);
        intakeMotor.setPower(intakePower);

        pivotServoLeft.setPosition(leftPosition);
        pivotServoRight.setPosition(rightPosition);
    }
}