package org.firstinspires.ftc.teamcode.subsystems;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.ftc12835.library.hardware.devices.REVHub;
import com.ftc12835.library.hardware.management.Subsystem;
import com.ftc12835.library.util.TelemetryUtil;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

public class MecanumDrive implements Subsystem {
    public static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    private Mode mode;

    private static final double kV = 0;
    private static final double kA = 0;
    private static final double kStatic = 0;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private double[] powers = new double[4];
    private MecanumPIDVAFollower trajectoryFollower;

    private TelemetryData telemetryData;

    private class TelemetryData {
        public double leftFrontPower;
        public double rightFrontPower;
        public double leftRearPower;
        public double rightRearPower;
    }

    private enum Mode {
        OPEN_LOOP,
        TRAJECTORY_FOLLOWING
    }

    public MecanumDrive(HardwareMap hardwareMap) {

        leftFront = hardwareMap.get(DcMotorEx.class, "FL");
        leftRear = hardwareMap.get(DcMotorEx.class, "BL");
        rightRear = hardwareMap.get(DcMotorEx.class, "BR");
        rightFront = hardwareMap.get(DcMotorEx.class, "FR");

        for (DcMotorEx motor : Arrays.asList(leftFront, leftRear, rightRear, rightFront)) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private static double encoderTicksToInches(int ticks) {
        return 4 * Math.PI * ticks / TICKS_PER_REV;
    }

    public MecanumPIDVAFollower getTrajectoryFollower() {
        return trajectoryFollower;
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        powers[0] = v;
        powers[1] = v1;
        powers[2] = v2;
        powers[3] = v3;
    }

    @NotNull
    public List<Double> getWheelPositions() {
        List<Double> positions = new ArrayList<>();
        positions.add(encoderTicksToInches(leftFront.getCurrentPosition()));
        positions.add(encoderTicksToInches(rightFront.getCurrentPosition()));
        positions.add(-encoderTicksToInches(leftRear.getCurrentPosition()));
        positions.add(-encoderTicksToInches(rightRear.getCurrentPosition()));

        return positions;
    }

    @Override
    public Map<String, Object> update(@Nullable Canvas fieldOverlay) {
        leftFront.setPower(powers[0]);
        rightFront.setPower(powers[1]);
        leftRear.setPower(powers[2]);
        rightRear.setPower(powers[4]);

        telemetryData.leftFrontPower = powers[0];
        telemetryData.rightFrontPower = powers[1];
        telemetryData.leftRearPower = powers[2];
        telemetryData.rightRearPower = powers[3];
        return TelemetryUtil.objectToMap(telemetryData);
    }
}
