package org.firstinspires.ftc.teamcode.opmodes;

import com.ftc12835.library.util.StickyGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name = "Main TeleOp", group = "COMPETITION")
public class MainTeleOp extends OpMode {
    private StickyGamepad stickyGamepad1, stickyGamepad2;
    private Robot robot;

    private boolean extenderReadyRoutineRunning = false;
    private boolean extenderStowRoutineRunning = false;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();

        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Ready");
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();

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

        if (gamepad1.y) {
            robot.latchingLift.setLiftPower(1.0);
        } else if (gamepad1.a) {
            robot.latchingLift.setLiftPower(-1.0);
        } else {
            robot.latchingLift.setLiftPower(0.0);
        }

        // ready routine
        if (gamepad1.left_bumper && !extenderReadyRoutineRunning) {
            extenderReadyRoutineRunning = true;
            robot.intake.setExtenderSetpoint(Intake.EXTENDER_READY_POSITION);
        }

        if (extenderReadyRoutineRunning && !robot.intake.isExtending()) {
            robot.intake.setIntakeSetpoint(Intake.IntakePosition.DEPLOY);
        }

        if (extenderReadyRoutineRunning && !robot.intake.isExtending() && !robot.intake.isDeploying()) {
            extenderReadyRoutineRunning = false;
        }

        // stow routine
        if (gamepad1.right_bumper && !extenderStowRoutineRunning) {
            extenderReadyRoutineRunning = true;
            robot.intake.setIntakeSetpoint(Intake.IntakePosition.RAISED);
        }

        if (extenderStowRoutineRunning && !robot.intake.isDeploying()) {
            robot.intake.setExtenderSetpoint(Intake.EXTENDER_STOW_POSITION);
        }

        if (extenderStowRoutineRunning && !robot.intake.isDeploying() && !robot.intake.isExtending()) {
            extenderReadyRoutineRunning = false;
        }
    }
}
