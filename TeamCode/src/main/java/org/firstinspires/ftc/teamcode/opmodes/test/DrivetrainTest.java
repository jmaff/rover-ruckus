package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.RobotDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Disabled
@TeleOp(name = "Drivetrain Testing", group = "TEST")
public class DrivetrainTest extends OpMode {
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor lift;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class,  "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        lift = hardwareMap.get(DcMotor.class, "LATCHING_LIFT");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    @Override
    public void loop() {
        // Determine the desired angle of where the driver wants to go based on the LEFT
        // joystick
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;

        // Calculate the motor values to achieve this desired angle, taking into account
        // the turn from the RIGHT joystick
        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;

        frontLeft.setPower(v1);
        frontRight.setPower(v2);
        backLeft.setPower(v3);
        backRight.setPower(v4);

        if (gamepad1.y) {
            lift.setPower(1.0);
        } else if (gamepad1.a) {
            lift.setPower(-1.0);
        } else {
            lift.setPower(0.0);
        }

//        if (gamepad1.a) {
//            frontLeft.setPower(1);
//        }
//        if (gamepad1.x) {
//            frontRight.setPower(1);
//        }
//        if (gamepad1.b) {
//            backLeft.setPower(1);
//        }
//        if (gamepad1.y) {
//            backRight.setPower(1);
//        }

    }

}
