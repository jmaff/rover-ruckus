package com.ftc12835.roverruckus.subsystems;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.MecanumPIDVAFollower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

public class MecanumDrive extends com.acmerobotics.roadrunner.drive.MecanumDrive implements Subsystem {
    public static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    public static final com.qualcomm.robotcore.hardware.PIDCoefficients NORMAL_VELOCITY_PID =
            new com.qualcomm.robotcore.hardware.PIDCoefficients(20, 8, 12);

    private static final PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
    private static final PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);

    private static final double kV = 0;
    private static final double kA = 0;
    private static final double kStatic = 0;

    private LynxModule frontHub;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private MecanumPIDVAFollower trajectoryFollower;

    public MecanumDrive(HardwareMap hardwareMap) {
        // lateral distance between pairs of wheels on different sides of the robot
        super(18.0);

        frontHub = hardwareMap.get(LynxModule.class, "frontHub");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        for (DcMotorEx motor : Arrays.asList(leftFront, leftRear, rightRear, rightFront)) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, NORMAL_VELOCITY_PID);
        }

        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        trajectoryFollower = new MecanumPIDVAFollower(
                this,
                TRANSLATIONAL_PID,
                HEADING_PID,
                kV,
                kA,
                kStatic);
    }

    private static double encoderTicksToInches(int ticks) {
        return 4 * Math.PI * ticks / TICKS_PER_REV;
    }

    public MecanumPIDVAFollower getTrajectoryFollower() {
        return trajectoryFollower;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(frontHub);
        List<Double> positions = new ArrayList<>();
        try {
            LynxGetBulkInputDataResponse response = command.sendReceive();
            positions.add(encoderTicksToInches(response.getEncoder(0)));
            positions.add(encoderTicksToInches(response.getEncoder(1)));
            positions.add(-encoderTicksToInches(response.getEncoder(2)));
            positions.add(-encoderTicksToInches(response.getEncoder(3)));
            return positions;
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (LynxNackException e) {
            // do something idk
        }
        return Arrays.asList(0.0, 0.0, 0.0, 0.0);
    }

    @Override
    public Map<String, Object> update(@Nullable Canvas fieldOverlay) {
        // TODO: Implement overlay to Dashboard
        return null;
    }
}
