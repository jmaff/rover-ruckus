package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Config
@Autonomous(name = "Crater Auto", group = "COMPETITION")
public class CraterAuto extends LinearOpMode {
    public static double TURN_SPEED_FAST = 0.37;
    public static double TURN_SPEED_ADJUST = 0.23;
    public static double TURN_AROUND_SPEED = 0.4;
    public static double FORWARD_SPEED = -0.7;
    public static double COLLECT_DRIVE_SPEED = -0.3;

    public static double STRAFE_X_SPEED = 0.8;
    public static double STRAFE_Y_SPEED = 0.1;

    public static int WAIT_TO_READ = 1200;

    /*
     * DEPLOYING
     */

    public static int LIFT_DOWN = 9800;
    // strafe off hook
    public static int STRAFE_OFF_HOOK = 140;
    // move forward away from lander
    public static int TO_TAPE = 190;

    /*
     * TEAM MARKER
     */
    public static double TURN_TO_WALL = 67;

    public static int TO_WALL = 900;

    public static double TURN_TO_DEPOT = 135;

    public static int TO_DEPOT = 300;

    public static int EXTEND_TO_DEPOT = 1650;

    public static int BACK_DEPOT = 255;

    public static double TURN_TO_RETURN = 62;

    public static int TO_SAMPLE = 900;

    /*
     * SAMPLING
     */
    public static double LEFT_TURN_SAMPLE = 32;
    public static double CENTER_TURN_SAMPLE = 0;
    public static double RIGHT_TURN_SAMPLE = -36;

    public static double WIGGLE_THRESHOLD = 12;
    public static double WIGGLE_TURN_CLAMP = 0.5;

    public static int EXTEND_TO_SAMPLE = 1000;

    public static double TURN_TO_SCORE = -2;

    // 140
    public static int BACK_TO_LANDER = 180;

    // 170
    public static int STRAFE_TO_SCORE = 250;

    public static int RAISE_TO_SCORE = 2200;
    public static double TIME_TO_TILT = 500;

    public static int LOWER = 70;

    /*
     * PARK
     */

    public static int EXTEND_TO_CRATER = 2200;

    public static int FORWARD_TO_SEARCH = 150;

    public static int MAX_EXTEND = 3000;


    private Robot robot;
    private Vision.GoldPosition goldPosition;

    private Runnable updateRunnable = () -> {
        while (opModeIsActive()) {
            robot.update();
            telemetry.addData("Heading", robot.mecanumDrive.getHeading());

            telemetry.update();
        }
    };

    private Thread updateThread = new Thread(updateRunnable);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.vision.enable();

        while (!opModeIsActive()) {
            robot.vision.update();

            switch (robot.vision.getGoldPosition()) {
                case LEFT:
                    telemetry.addData("Gold Position", "LEFT");
                    break;
                case CENTER:
                    telemetry.addData("Gold Position", "CENTER");
                    break;
                case RIGHT:
                    telemetry.addData("Gold Position", "RIGHT");
                    break;
                case UNKNOWN:
                    telemetry.addData("Gold Position", "UNKNOWN");
                    break;
                case ERROR:
                    telemetry.addData("Gold Position", "ERROR! ERROR! ERROR! Call the FTA's! RUN!!!");
                    break;
            }
            telemetry.update();
        }

        boolean safeSample = false;

        waitForStart();

        if (robot.vision.getGoldPosition() == Vision.GoldPosition.LEFT) safeSample = true;

        if (!safeSample) {
            double leftCount = 0;
            double centerCount = 0;
            double rightCount = 0;

            for (int i = 0; i < 10; i++) {
                if (robot.vision.getGoldPosition() == Vision.GoldPosition.LEFT) {
                    leftCount++;
                } else if (robot.vision.getGoldPosition() == Vision.GoldPosition.CENTER) {
                    centerCount++;
                } else {
                    rightCount++;
                }
                sleep(50);
            }

            goldPosition = (leftCount > centerCount && leftCount > rightCount) ? Vision.GoldPosition.LEFT :
                    (centerCount > rightCount && centerCount > leftCount) ? Vision.GoldPosition.CENTER :
                            (rightCount > leftCount && rightCount > centerCount) ? Vision.GoldPosition.RIGHT : Vision.GoldPosition.CENTER;

            robot.vision.disable();
        }

        robot.start();
        updateThread.start();

        robot.mecanumDrive.brakeMode(true);

        robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
        robot.latchingLift.runLiftToPosition(1.0, LIFT_DOWN);
        sleep(500);

        robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        if (safeSample) {
            sleep(WAIT_TO_READ);
            double leftCount = 0;
            double centerCount = 0;
            double rightCount = 0;

            for (int i = 0; i < 10; i++) {
                if (robot.vision.getGoldPosition() == Vision.GoldPosition.LEFT) {
                    leftCount++;
                } else if (robot.vision.getGoldPosition() == Vision.GoldPosition.CENTER) {
                    centerCount++;
                } else {
                    rightCount++;
                }
                sleep(50);
            }

            goldPosition = (leftCount > centerCount && leftCount > rightCount) ? Vision.GoldPosition.LEFT :
                    (centerCount > rightCount && centerCount > leftCount) ? Vision.GoldPosition.CENTER :
                            (rightCount > leftCount && rightCount > centerCount) ? Vision.GoldPosition.RIGHT : Vision.GoldPosition.CENTER;

            robot.vision.disable();
        }

        robot.mecanumDrive.singleEncoderDrive(STRAFE_X_SPEED, STRAFE_Y_SPEED, 0, STRAFE_OFF_HOOK);
        sleep(100);

        robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, TO_TAPE);
        sleep(100);

        robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, TURN_TO_WALL);

        robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, TO_WALL);
        sleep(100);

        robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, TURN_TO_DEPOT);

        robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, TO_DEPOT);
        sleep(100);

        robot.intake.runExtenderToPosition(-1.0, EXTEND_TO_DEPOT);

        robot.intake.dumpMarker();

        robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);

        robot.intake.retractIntakeExtender();

        robot.intake.setIntakePivotPosition(Intake.PivotPosition.UP);

        robot.mecanumDrive.encoderDrive(0, -FORWARD_SPEED, 0, BACK_DEPOT);
        sleep(100);

        robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, TURN_TO_RETURN);

        robot.mecanumDrive.encoderDrive(0, -FORWARD_SPEED, 0, TO_SAMPLE);
        sleep(100);

        robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);

        switch (goldPosition) {
            case LEFT:
                robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, LEFT_TURN_SAMPLE);
                break;
            default:
            case CENTER:
                robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, CENTER_TURN_SAMPLE);
                break;
            case RIGHT:
                robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, RIGHT_TURN_SAMPLE);
                break;
        }

        robot.intake.setIntakePower(-1.0);
        robot.intake.setIntakePivotPosition(Intake.PivotPosition.DOWN);
        sleep(300);

        robot.intake.setIntakePower(-1.0);

        robot.intake.runExtenderToPosition(-1.0, EXTEND_TO_SAMPLE);
        if (goldPosition == Vision.GoldPosition.RIGHT) {
            robot.intake.runExtenderToPosition(-1.0, EXTEND_TO_SAMPLE + 200);
        }
        sleep(700);

        boolean mineralCollected = false;
        for (int i = 0; i < 5; i++) {
            if (robot.intake.getMineralStatus() != Intake.MineralStatus.NONE) {
                mineralCollected = true;
                break;
            }
            sleep(75);
        }

        if (!mineralCollected) {
            switch (goldPosition) {
                case LEFT:
                    robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, LEFT_TURN_SAMPLE + WIGGLE_THRESHOLD, WIGGLE_TURN_CLAMP);
                    robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, LEFT_TURN_SAMPLE - WIGGLE_THRESHOLD, WIGGLE_TURN_CLAMP);
                    break;
                default:
                case CENTER:
                    robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, CENTER_TURN_SAMPLE + WIGGLE_THRESHOLD, WIGGLE_TURN_CLAMP);
                    robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, CENTER_TURN_SAMPLE - WIGGLE_THRESHOLD, WIGGLE_TURN_CLAMP);
                    break;
                case RIGHT:
                    robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, RIGHT_TURN_SAMPLE + WIGGLE_THRESHOLD, WIGGLE_TURN_CLAMP);
                    robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, RIGHT_TURN_SAMPLE - WIGGLE_THRESHOLD, WIGGLE_TURN_CLAMP);
                    break;
            }
            robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);
            sleep(500);
            robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, TURN_TO_SCORE);
        } else {
            robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);
            robot.intake.retractIntakeExtender();
            robot.intake.setIntakePivotPosition(Intake.PivotPosition.UP);

            sleep(900);

            robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);
            sleep(300);
            robot.intake.setIntakePower(0.0);

            robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, TURN_TO_SCORE);

            robot.mecanumDrive.encoderDrive(0, -FORWARD_SPEED, 0, BACK_TO_LANDER);
            sleep(100);

            robot.mecanumDrive.encoderDrive(0.8, 0, 0, STRAFE_TO_SCORE);
            sleep(100);

            robot.outtake.runLiftToPosition(-1.0, RAISE_TO_SCORE);

            robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);

            robot.outtake.setOuttakePosition(Outtake.OuttakePosition.TILT);
            sleep((long)TIME_TO_TILT);
            robot.outtake.setOuttakePosition(Outtake.OuttakePosition.UP);

            sleep(800);
            robot.outtake.setOuttakePosition(Outtake.OuttakePosition.DOWN);
        }

        robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);
        sleep(700);
        robot.intake.runExtenderToPosition(-1.0, EXTEND_TO_CRATER);
        robot.intake.setIntakePivotPosition(Intake.PivotPosition.DOWN);

        robot.mecanumDrive.resetEncoders();

        robot.intake.setIntakePower(-1.0);

        boolean keepDriving = true;
        while (opModeIsActive() && robot.intake.getMineralStatus() != Intake.MineralStatus.TWO) {
            for (int position : robot.mecanumDrive.getWheelPositions()) {
                if (Math.abs(position) > FORWARD_TO_SEARCH) {
                    keepDriving = false;
                    robot.mecanumDrive.stop();
                }
            }

            if (keepDriving) {
                robot.mecanumDrive.cartesianDrive(0, COLLECT_DRIVE_SPEED, 0);
            }

            if (robot.intake.getExtenderPosition() < MAX_EXTEND) {
                robot.intake.setExtenderPower(-1.0);
            } else {
                robot.intake.setExtenderPower(0.0);
            }
        }

        robot.stop();
        robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);


    }
}
