package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.RobotDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

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

    private double integratedZAxis = 0;
    private double lastHeading = 0;

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

        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");

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

        // Determine the desired angle of where the driver wants to go based on the LEFT
        // joystick
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        // Calculate the motor values to achieve this desired angle, taking into account
        // the turn from the RIGHT joystick
//        double v1 = r * Math.cos(robotAngle) + rightX;
//        double v2 = r * Math.sin(robotAngle) - rightX;
//        double v3 = r * Math.sin(robotAngle) + rightX;
//        double v4 = r * Math.cos(robotAngle) - rightX;
//
//        left1.setPower(v1);
//        left2.setPower(v2);
//        right2.setPower(v3);
//        right1.setPower(v4);

        if (gamepad1.a) {
            left1.setPower(1);
        }
        if (gamepad1.x) {
            left2.setPower(1);
        }
        if (gamepad1.b) {
            right1.setPower(1);
        }
        if (gamepad1.y) {
            right2.setPower(1);
        }

        Telemetry dashboardTelemetry = RobotDashboard.getInstance().getTelemetry();
        dashboardTelemetry.addData("time", timer.seconds());
        dashboardTelemetry.addData("heading", getIntegratedZAxis());
        dashboardTelemetry.addData("raw heading", imu.getAngularOrientation().firstAngle);
        dashboardTelemetry.update();
    }

    double getIntegratedZAxis() {
        double newHeading = imu.getAngularOrientation().firstAngle;
        double deltaHeading = newHeading - lastHeading;
        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if(deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        integratedZAxis += deltaHeading;
        lastHeading = newHeading;
        return integratedZAxis;
    }
}
