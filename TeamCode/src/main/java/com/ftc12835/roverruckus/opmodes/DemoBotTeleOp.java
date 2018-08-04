package com.ftc12835.roverruckus.opmodes;

import com.acmerobotics.dashboard.RobotDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Demo Bot TeleOp", group = "PID")
public class DemoBotTeleOp extends OpMode {
    private ElapsedTime timer = new ElapsedTime();
    private RobotDashboard dashboard;
    private BNO055IMU imu;
    DcMotor left1;
    DcMotor left2;
    DcMotor right1;
    DcMotor right2;
    DcMotor center;
    Servo arm;

    @Override
    public void init() {
        left1 = hardwareMap.get(DcMotor.class,  "L1");
        left2 = hardwareMap.get(DcMotor.class, "L2");
        right1 = hardwareMap.get(DcMotor.class, "R1");
        right2 = hardwareMap.get(DcMotor.class, "R2");
        center = hardwareMap.get(DcMotor.class, "C");
        arm = hardwareMap.get(Servo.class, "ARM");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        dashboard = RobotDashboard.getInstance();
        dashboard.updateConfig();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        double turn = gamepad1.right_stick_x;
        double xSpeed = gamepad1.left_stick_y;

        left1.setPower(-(xSpeed + turn));
        left2.setPower((xSpeed + turn));
        right1.setPower((xSpeed - turn));
        right2.setPower(-(xSpeed - turn));

        center.setPower(gamepad1.left_stick_x);


        Telemetry dashboardTelemetry = RobotDashboard.getInstance().getTelemetry();
        dashboardTelemetry.addData("time", timer.seconds());
        dashboardTelemetry.addData("heading", imu.getAngularOrientation().firstAngle);
        dashboardTelemetry.update();
    }
}
