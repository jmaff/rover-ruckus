package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Testing TeleOp", group = "TEST")
public class TestTeleOp extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.intake.setMode(Intake.Mode.OPEN_LOOP);
    }

    @Override
    public void start() {
        robot.start();
        super.start();
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

        if (gamepad1.y) {
            robot.intake.pivotServoLeft.setPosition(0.0);
            robot.intake.pivotServoRight.setPosition(1.0);
        } else if (gamepad1.a) {
            robot.intake.pivotServoLeft.setPosition(1.0);
            robot.intake.pivotServoRight.setPosition(0.0);
        }

        if (gamepad1.right_bumper) {
            robot.intake.setIntakePower(1.0);
        } else if (gamepad1.left_bumper) {
            robot.intake.setIntakePower(-1.0);
        } else {
            robot.intake.setIntakePower(0.0);
        }
    }
}
