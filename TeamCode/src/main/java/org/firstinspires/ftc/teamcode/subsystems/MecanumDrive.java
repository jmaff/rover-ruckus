package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc12835.library.hardware.management.Subsystem;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

    private DcMotor leftFront, leftRear, rightRear, rightFront;
    private BNO055IMU imu;
    private double[] powers = new double[4];

    private OpMode opMode;

    private double lastHeading = 0;
    private double integratedZAxis = 0;

    public MecanumDrive(OpMode opMode) {
        this.opMode = opMode;

        leftFront = opMode.hardwareMap.get(DcMotor.class, "FL");
        leftRear = opMode.hardwareMap.get(DcMotor.class, "BL");
        rightRear = opMode.hardwareMap.get(DcMotor.class, "BR");
        rightFront = opMode.hardwareMap.get(DcMotor.class, "FR");
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");

        for (DcMotor motor : Arrays.asList(leftFront, leftRear, rightRear, rightFront)) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
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

            for (int position : getWheelPositions()) {
                if (Math.abs(position) > counts) {
                    stop();
                    return;
                }
            }
        }

        stop();
    }

    public List<Integer> getWheelPositions() {
        List<Integer> positions = new ArrayList<>();
        positions.add(leftFront.getCurrentPosition());
        positions.add(rightFront.getCurrentPosition());
        positions.add(leftRear.getCurrentPosition());
        positions.add(rightRear.getCurrentPosition());

        return positions;
    }

    public void stop() {
        setMotorPowers(0 , 0, 0, 0);
    }

    void updateIntegratedZAxis() {
        double newHeading = imu.getAngularOrientation().firstAngle;
        double deltaHeading = newHeading - lastHeading;
        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if(deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        integratedZAxis += deltaHeading;
        lastHeading = newHeading;
    }

    public double getHeading() {
        return integratedZAxis;
    }

    @Override
    public void update() {
        leftFront.setPower(powers[0]);
        rightFront.setPower(powers[1]);
        leftRear.setPower(powers[2]);
        rightRear.setPower(powers[3]);

        updateIntegratedZAxis();

        Telemetry telemetry = opMode.telemetry;
        telemetry.addData("FL", leftFront.getCurrentPosition());
        telemetry.addData("FR", rightFront.getCurrentPosition());
        telemetry.addData("BL", leftRear.getCurrentPosition());
        telemetry.addData("BR", rightRear.getCurrentPosition());
    }
}
