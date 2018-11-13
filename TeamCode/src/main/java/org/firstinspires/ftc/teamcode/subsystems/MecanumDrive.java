package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc12835.library.hardware.management.Subsystem;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class MecanumDrive implements Subsystem {
    public static final MotorConfigurationType MOTOR_CONFIG = MotorConfigurationType.getMotorType(NeveRest20Gearmotor.class);
    private static final double TICKS_PER_REV = MOTOR_CONFIG.getTicksPerRev();

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private double[] powers = new double[4];

    private OpMode opMode;

    public MecanumDrive(OpMode opMode) {
        this.opMode = opMode;

        leftFront = opMode.hardwareMap.get(DcMotorEx.class, "FL");
        leftRear = opMode.hardwareMap.get(DcMotorEx.class, "BL");
        rightRear = opMode.hardwareMap.get(DcMotorEx.class, "BR");
        rightFront = opMode.hardwareMap.get(DcMotorEx.class, "FR");

        for (DcMotorEx motor : Arrays.asList(leftFront, leftRear, rightRear, rightFront)) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private static double encoderTicksToInches(int ticks) {
        return 4 * Math.PI * ticks / TICKS_PER_REV;
    }

    public void setMotorPowers(double v, double v1, double v2, double v3) {
        powers[0] = v;
        powers[1] = v1;
        powers[2] = v2;
        powers[3] = v3;
    }

    public void cartesianDrive(double x, double y, double turn) {
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, x) - Math.PI / 4;

        double v1 = r * Math.cos(robotAngle) + turn;
        double v2 = r * Math.sin(robotAngle) - turn;
        double v3 = r * Math.sin(robotAngle) + turn;
        double v4 = r * Math.cos(robotAngle) - turn;

        setMotorPowers(v1, v2, v3, v4);
    }

    public void encoderDrive(double x, double y, double turn, int counts) {
        LinearOpMode linearOpMode = (LinearOpMode) opMode;
        cartesianDrive(x, y, turn);

        while (linearOpMode.opModeIsActive()) {
            for (double position : getWheelPositions()) {
                if (Math.abs(position) > counts) {
                    break;
                }
            }
        }

        stop();
    }

    public List<Double> getWheelPositions() {
        List<Double> positions = new ArrayList<>();
        positions.add(encoderTicksToInches(leftFront.getCurrentPosition()));
        positions.add(encoderTicksToInches(rightFront.getCurrentPosition()));
        positions.add(-encoderTicksToInches(leftRear.getCurrentPosition()));
        positions.add(-encoderTicksToInches(rightRear.getCurrentPosition()));

        return positions;
    }

    public void stop() {
        setMotorPowers(0 , 0, 0, 0);
    }

    @Override
    public void update() {
        leftFront.setPower(powers[0]);
        rightFront.setPower(powers[1]);
        leftRear.setPower(powers[2]);
        rightRear.setPower(powers[3]);

        Telemetry telemetry = opMode.telemetry;
        telemetry.addData("FL", leftFront.getCurrentPosition());
        telemetry.addData("FR", rightFront.getCurrentPosition());
        telemetry.addData("BL", leftRear.getCurrentPosition());
        telemetry.addData("BR", rightRear.getCurrentPosition());
    }
}
