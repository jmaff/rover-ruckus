package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp(name = "Main TeleOp", group = "COMPETITION")
public class MainTeleOp extends OpMode {

    // automatic dumping variables
    private boolean intakePrev = false;
    private long timeIntakeDumped = 0;
    private boolean dumping = false;

    private long timeLowerTriggered = 0;
    private boolean lowering = false;

    private boolean retracting = false;

    // mineral detection variables
    private Intake.MineralStatus prevMineralStatus = Intake.MineralStatus.NONE;
    private long timeDetected = 0;

    private Robot robot;

    public static double SLOW_INTAKE_SPEED = -0.2;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();
        robot.mecanumDrive.brakeMode(false);
    }

    @Override
    public void start() {
        robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    @Override
    public void loop() {
        // drive
        robot.mecanumDrive.cartesianDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, 0.5 * gamepad1.right_stick_x);

        // intake extender controls
        if (gamepad2.dpad_right) {
            robot.intake.setExtenderPower(1.0);
            retracting = false;
        } else if (gamepad2.dpad_left) {
            robot.intake.setExtenderPower(-1.0);
            retracting = false;
        } else if (!retracting) {
            robot.intake.setExtenderPower(0.0);
        } else if (retracting) {
            robot.intake.setExtenderPower(1.0);
        }

//        if (gamepad2.right_trigger != 0) {
//            robot.intake.setIntakePivotPosition(Intake.PivotPosition.HIGH);
//            robot.intake.setIntakePower(SLOW_INTAKE_SPEED);
//            robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//            robot.intake.setExtenderPower(1.0);
//            retracting = true;
//        }

        // intake motor controls
        if (gamepad2.left_bumper) {
            robot.intake.setIntakePower(-1.0);
        } else if (gamepad2.right_bumper) {
            robot.intake.setIntakePower(1.0);
        } else if (gamepad2.start){
            robot.intake.setIntakePower(0.0);
        }

        // mineral lift controls
        if (gamepad1.left_trigger != 0) {
           lowering = false;
           robot.outtake.setLiftPower(1.0);
           robot.outtake.setOuttakePosition(Outtake.OuttakePosition.DOWN);
           robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (gamepad1.right_trigger != 0) {
            lowering = false;
            robot.outtake.setLiftPower(-1.0);
        } else if (!lowering) {
            robot.outtake.setLiftPower(0.0);
        }

        if (gamepad1.a) {
            lowering = true;
            robot.outtake.setOuttakePosition(Outtake.OuttakePosition.DOWN);
            robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            timeLowerTriggered = System.currentTimeMillis();
        }

        if (lowering) {
            if (robot.outtake.getOuttakeLimit() || System.currentTimeMillis() - timeLowerTriggered >= 2000) {
                robot.outtake.setLiftPower(0.0);
                lowering = false;
            } else {
                robot.outtake.setLiftPower(1.0);
            }
        }

        if (!gamepad2.x && !gamepad2.b) {
            if (((robot.intake.getIntakeLimit() && !intakePrev) || gamepad2.right_trigger != 0) && !dumping) {
                dumping = true;
                robot.intake.setIntakePower(-1.0);
                robot.intake.setIntakePivotPosition(Intake.PivotPosition.UP);
                timeIntakeDumped = System.currentTimeMillis();
            }

            if (dumping && System.currentTimeMillis() - timeIntakeDumped >= 1100) {
                robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);
                robot.intake.setIntakePower(0.0);
                dumping = false;
                timeIntakeDumped = 0;
            }
        }

        intakePrev = robot.intake.getIntakeLimit();

        if (dumping) retracting = false;

        // intake state controls
        if (gamepad2.b) {
            robot.intake.setIntakePivotPosition(Intake.PivotPosition.UP);
            robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (gamepad2.x) {
            robot.intake.setIntakePivotPosition(Intake.PivotPosition.DOWN);
            robot.intake.setIntakePower(-1.0);
            robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (gamepad2.dpad_right && !dumping && !retracting) {
            robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);
            robot.intake.setIntakePower(SLOW_INTAKE_SPEED);
            robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }

        Intake.MineralStatus status = robot.intake.getMineralStatus();

        if (status != prevMineralStatus && status == Intake.MineralStatus.TWO) {
            timeDetected = System.currentTimeMillis();
        }

        if (timeDetected != 0 && System.currentTimeMillis() - timeDetected >= 900) {
            robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_FAST);
            timeDetected = 0;
        }

        prevMineralStatus = status;

        // outtake controls
        if (gamepad2.dpad_up) {
            robot.outtake.setOuttakePosition(Outtake.OuttakePosition.UP);
            robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
        } else if (gamepad2.dpad_down) {
            robot.outtake.setOuttakePosition(Outtake.OuttakePosition.DOWN);
            robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }

        // latching lift controls
        if (gamepad1.right_bumper) {
            robot.latchingLift.setLiftPower(1.0);
        } else if (gamepad1.left_bumper) {
            robot.latchingLift.setLiftPower(-1.0);
            robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
        } else {
            robot.latchingLift.setLiftPower(0.0);
        }

        telemetry.addData("Latching Lift Position: ", robot.latchingLift.getEncoderCounts());
        telemetry.addData("Extension Position: ", robot.intake.getExtenderPosition());

        robot.update();
    }
}
