package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.ftc12835.library.hardware.management.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Outtake implements Subsystem {
    private DcMotor mineralLift;
    private Servo leftPivot;
    private Servo rightPivot;

    private DigitalChannel outtakeLimit;

    private double liftPower;

    private OpMode opMode;

    public static double UP = 0.47;
    public static double DOWN = 0.9;
    private double pivotPosition;

    public enum OuttakePosition {
        UP,
        DOWN
    }

    public Outtake(OpMode opMode) {
        mineralLift = opMode.hardwareMap.get(DcMotor.class, "MINERAL_LIFT");
        leftPivot = opMode.hardwareMap.get(Servo.class, "OUTTAKE_LEFT");
        rightPivot = opMode.hardwareMap.get(Servo.class, "OUTTAKE_RIGHT");
        outtakeLimit = opMode.hardwareMap.get(DigitalChannel.class, "OUTTAKE_LIMIT");
        mineralLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setOuttakePosition(OuttakePosition.DOWN);

        resetLiftEncoder();

        this.opMode = opMode;
    }

    public void setLiftPower(double liftPower) {
        this.liftPower = liftPower;
    }

    public void setRawOuttakePosition(double pivotPosition) {
        this.pivotPosition = pivotPosition;
    }

    public void setOuttakePosition(OuttakePosition outtakePosition) {
        switch (outtakePosition) {
            case UP:
                setRawOuttakePosition(UP);
                break;
            case DOWN:
                setRawOuttakePosition(DOWN);
                break;
        }
    }

    public int getLiftPosition() {
        return -mineralLift.getCurrentPosition();
    }

    public void resetLiftEncoder() {
        mineralLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mineralLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runLiftToPosition(double power, int counts) {
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        setLiftPower(power);

        while (linearOpMode.opModeIsActive()) {
            if (Math.abs(getLiftPosition()) > counts) {
                break;
            }
        }

        setLiftPower(0.0);
    }

    public void lowerLiftToPosition(double power, int counts) {
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        setLiftPower(power);

        while (linearOpMode.opModeIsActive()) {
            if (Math.abs(getLiftPosition()) < counts) {
                break;
            }
        }

        setLiftPower(0.0);
    }

    public boolean getOuttakeLimit() {
        return !outtakeLimit.getState();
    }

    @Override
    public void update() {
        mineralLift.setPower(liftPower);

        leftPivot.setPosition(pivotPosition);
        rightPivot.setPosition(1.00 - pivotPosition);
    }
}
