package org.firstinspires.ftc.teamcode.subsystems;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.ftc12835.library.hardware.management.Subsystem;
import com.ftc12835.library.util.TelemetryUtil;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

@Config
public class LatchingLift implements Subsystem {

    private DcMotor liftMotor;
    private double liftPower;
    public static int DEPLOY_POSITION = 0;
    private OpMode opMode;

    public LatchingLift(OpMode opMode) {
        this.opMode = opMode;
        liftMotor = opMode.hardwareMap.get(DcMotor.class, "LATCHING_LIFT");
        resetEncoder();
    }

    public void setLiftPower(double power) {
        liftPower = power;
    }

    public void resetEncoder() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getEncoderCounts() {
        return liftMotor.getCurrentPosition();
    }

    public void runLiftToPosition(double power, int counts) {
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        setLiftPower(power);

        while (linearOpMode.opModeIsActive()) {
            if (Math.abs(getEncoderCounts()) > counts) {
                break;
            }
        }

        setLiftPower(0.0);
    }

    @Override
    public void update() {
        liftMotor.setPower(liftPower);
        opMode.telemetry.addData("Lift Count", getEncoderCounts());
    }
}
