package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.yaml.snakeyaml.events.Event;

@Disabled
@TeleOp(name = "Testing TeleOp", group = "TEST")
public class TestTeleOp extends OpMode {
    public static double LEFT_DOWN = 1.0;
    public static double RIGHT_DOWN = 0.0;
    public static double LEFT_UP = 0.0;
    public static double RIGHT_UP = 1.0;
    public static double IDLE = 0.5;
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();
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

        robot.mecanumDrive.setMotorPowers(v1, v2, v3, v4);

        if (gamepad1.dpad_up) {
            robot.intake.setExtenderPower(1.0);
        } else if (gamepad1.dpad_down) {
            robot.intake.setExtenderPower(-1.0);
        } else {
            robot.intake.setExtenderPower(0.0);
        }

        if (gamepad1.right_bumper) {
            robot.intake.setIntakePower(1.0);
        } else if (gamepad1.left_bumper) {
            robot.intake.setIntakePower(-1.0);
        } else {
            robot.intake.setIntakePower(0.0);
        }

        if (gamepad1.y) {
            robot.latchingLift.setLiftPower(1.0);
        } else if (gamepad1.a) {
            robot.latchingLift.setLiftPower(-1.0);
        } else {
            robot.latchingLift.setLiftPower(0.0);
        }

        if (gamepad1.x) {
            robot.intake.setLeftPosition(LEFT_DOWN);
            robot.intake.setRightPosition(RIGHT_DOWN);
        } else if (gamepad1.b) {
            robot.intake.setLeftPosition(LEFT_UP);
            robot.intake.setRightPosition(RIGHT_UP);
        } else if (gamepad1.dpad_left) {
            robot.intake.setLeftPosition(IDLE);
            robot.intake.setRightPosition(IDLE);
        }

        robot.update();
    }
}
