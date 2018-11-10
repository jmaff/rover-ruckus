package com.ftc12835.library.hardware.drive;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.ftc12835.library.hardware.management.RobotTemplate;
import com.ftc12835.library.hardware.management.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.Map;

public abstract class MecanumDrive extends com.acmerobotics.roadrunner.drive.MecanumDrive implements Subsystem {
    private RobotTemplate robot;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private DcMotor.RunMode mode;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior;

    private static final double ENCODER_SCALAR = 1.0;
    private static final double WHEEL_RADIUS = 2.0;

    public MecanumDrive(double trackWidth, double wheelBase) {
        super(trackWidth, wheelBase);
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {

    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        return null;
    }

    @Override
    public void update() {
    }
}
