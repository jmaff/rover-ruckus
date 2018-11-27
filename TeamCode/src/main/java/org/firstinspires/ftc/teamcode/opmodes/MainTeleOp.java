package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.ftc12835.library.util.StickyGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp(name = "Main TeleOp", group = "COMPETITION")
public class MainTeleOp extends OpMode {

    private static double OUTTAKE_UP = 1.0;
    private static double OUTTAKE_DOWN = 0.0;


    private boolean bPrev = false;
    private boolean xPrev = false;

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();
        robot.mecanumDrive.brakeMode(false);
    }

    @Override
    public void loop() {
        // drive
        robot.mecanumDrive.cartesianDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        // intake extender controls
        if (gamepad2.dpad_left) {
            robot.intake.setExtenderPower(-1.0);
        } else if (gamepad2.dpad_right) {
            robot.intake.setExtenderPower(1.0);
        } else {
            robot.intake.setExtenderPower(0.0);
        }

        // intake motor controls
        if (gamepad2.left_bumper) {
            robot.intake.setIntakePower(-1.0);
        } else if (gamepad2.right_bumper) {
            robot.intake.setIntakePower(1.0);
        } else {
            robot.intake.setIntakePower(0.0);
        }

        // mineral lift controls
        if (gamepad2.y) {
           robot.outtake.setLiftPower(1.0);
        } else if (gamepad2.a) {
            robot.outtake.setLiftPower(-1.0);
        } else {
            robot.outtake.setLiftPower(0.0);
        }

        // intake state controls
        if (gamepad2.b) {
            robot.intake.setIntakePivotPosition(Intake.PivotPosition.DEPLOY);
        } else if (gamepad2.x) {
            robot.intake.setIntakePivotPosition(Intake.PivotPosition.STOW);
        } else if (gamepad2.start) {
            robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);
        }

//        if (gamepad2.b && !bPrev) {
//            if (robot.intake.getCurrentPivotPosition() == Intake.PivotPosition.MIDDLE) {
//                robot.intake.setIntakePivotPosition(Intake.PivotPosition.DEPLOY);
//            } else if (robot.intake.getCurrentPivotPosition() == Intake.PivotPosition.STOW) {
//                robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);
//            }
//        } else if (gamepad2.x && !xPrev) {
//            if (robot.intake.getCurrentPivotPosition() == Intake.PivotPosition.MIDDLE) {
//                robot.intake.setIntakePivotPosition(Intake.PivotPosition.STOW);
//            } else if (robot.intake.getCurrentPivotPosition() == Intake.PivotPosition.DEPLOY) {
//                robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);
//            }
//        }

        // outtake controls
        if (gamepad2.dpad_up) {
            robot.outtake.setOuttakePosition(OUTTAKE_UP);
        } else if (gamepad2.dpad_down) {
            robot.outtake.setOuttakePosition(OUTTAKE_DOWN);
        }

        // latching lift controls
        if (gamepad1.right_bumper) {
            robot.latchingLift.setLiftPower(1.0);
        } else if (gamepad1.left_bumper) {
            robot.latchingLift.setLiftPower(-1.0);
        } else {
            robot.latchingLift.setLiftPower(0.0);
        }

        robot.update();

        bPrev = gamepad1.b;
        xPrev = gamepad1.x;
    }
}
