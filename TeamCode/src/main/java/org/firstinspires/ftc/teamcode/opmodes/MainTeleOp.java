package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.ftc12835.library.util.StickyGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp(name = "Main TeleOp", group = "COMPETITION")
public class MainTeleOp extends OpMode {
    public static double LEFT_DOWN_FULL = 1.0;
    public static double RIGHT_DOWN_FULL = 0.0;
    public static double LEFT_DOWN_IDLE = 0.6;
    public static double RIGHT_DOWN_IDLE = 0.4;

    public static double LEFT_UP_FULL = 0.0;
    public static double RIGHT_UP_FULL = 1.0;
    public static double LEFT_UP_IDLE = 0.4;
    public static double RIGHT_UP_IDLE = 0.6;

    public static double NEUTRAL = 0.5;

    private static double OUTTAKE_UP = 1.0;
    private static double OUTTAKE_DOWN = 0.0;

    private Robot robot;

    /**
     * -2: STOW IDLE
     * -1: STOW MOVE
     * 0: NEUTRAL
     * 1: DEPLOYED MOVE
     * 2: DEPLOYED IDLE
     */
    private int intakeState = -2;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();
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
            if (intakeState == -2) {
                intakeState += 2;
            } else {
                intakeState++;
            }
        } else if (gamepad2.x) {
            if (intakeState == 2) {
                intakeState -= 2;
            } else {
                intakeState--;
            }
        }

        // set intake position based on state
        if (intakeState < -2) intakeState = -2;
        if (intakeState > 2) intakeState = 2;

        switch (intakeState) {
            case -2:
                robot.intake.setLeftPosition(LEFT_UP_IDLE);
                robot.intake.setRightPosition(RIGHT_UP_IDLE);
                break;
            case -1:
                robot.intake.setLeftPosition(LEFT_UP_FULL);
                robot.intake.setRightPosition(RIGHT_UP_FULL);
                break;
            case 0:
                robot.intake.setLeftPosition(NEUTRAL);
                robot.intake.setRightPosition(NEUTRAL);
                break;
            case 1:
                robot.intake.setLeftPosition(LEFT_DOWN_FULL);
                robot.intake.setRightPosition(RIGHT_DOWN_FULL);
                break;
            case 2:
                robot.intake.setLeftPosition(LEFT_DOWN_IDLE);
                robot.intake.setRightPosition(RIGHT_DOWN_IDLE);
                break;
        }

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
            robot.intake.setExtenderPower(-1.0);
        } else {
            robot.intake.setExtenderPower(0.0);
        }

        robot.update();
    }
}
