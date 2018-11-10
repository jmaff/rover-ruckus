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

    private DcMotor liftMotor;
    private double liftPower;

    public LatchingLift(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotor.class, "LATCHING_LIFT");
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLiftPower(double power) {
        liftPower = power;
    }

    public void resetEncoder() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        liftMotor.setPower(liftPower);
    }
}
