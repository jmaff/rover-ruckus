package com.ftc12835.roverruckus.opmodes;

import com.acmerobotics.dashboard.RobotDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.library.util.CSVWriter;
import com.acmerobotics.library.util.LoggingUtil;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;


@TeleOp(name = "Proportional Control", group = "PID")
@Config
public class ProportionalControl extends LinearOpMode {
    public static double kP = 0.1;

    private ElapsedTime timer = new ElapsedTime();
    private RobotDashboard dashboard;
    private BNO055IMU imu;
    DcMotor left1;
    DcMotor left2;
    DcMotor right1;
    DcMotor right2;
    DcMotor center;
    Servo arm;

    double output;
    double error;
    public static final double setpoint = 90.0;

    @Override
    public void runOpMode() throws InterruptedException {
        left1 = hardwareMap.get(DcMotor.class,  "L1");
        left2 = hardwareMap.get(DcMotor.class, "L2");
        right1 = hardwareMap.get(DcMotor.class, "R1");
        right2 = hardwareMap.get(DcMotor.class, "R2");
        center = hardwareMap.get(DcMotor.class, "C");
        arm = hardwareMap.get(Servo.class, "ARM");

        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        dashboard = RobotDashboard.getInstance();
        dashboard.updateConfig();

        Telemetry dashboardTelemetry = RobotDashboard.getInstance().getTelemetry();

        File logRoot = LoggingUtil.getLogRoot(this);
        String prefix = "ProportionalControl" + System.currentTimeMillis();
        CSVWriter writer = new CSVWriter(new File(logRoot, prefix + ".csv"));

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            error = setpoint - getHeading();
            output = kP * error;

            left1.setPower(output);
            left2.setPower(output);
            right1.setPower(-output);
            right2.setPower(-output);

            writer.put("timer", timer.seconds());
            writer.put("error", error);
            writer.put("output", output);
            writer.put("heading", getHeading());
            writer.write();

            dashboardTelemetry.addData("time", timer.seconds());
            dashboardTelemetry.addData("error", error);
            dashboardTelemetry.addData("output", output);
            dashboardTelemetry.addData("heading", getHeading());
            dashboardTelemetry.update();
        }

        left1.setPower(0);
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);
        writer.close();
    }

    public double getHeading() {
        if (imu.getAngularOrientation().firstAngle < 0) {
            return 360.0 + imu.getAngularOrientation().firstAngle;
        }
        return imu.getAngularOrientation().firstAngle;

    }
}
