package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
    public static double FORWARD_SPEED = -0.85;
    public static double X_OFFSET_SPEED = 0.2;

    /*
     * DEPLOYING
     */

    public static int LIFT_DOWN = 9800;
    // strafe off hook
    public static int STRAFE_OFF_HOOK = 100;
    // move forward away from lander
    public static int TO_TAPE = 160;

    /*
     * TEAM MARKER
     */
    public static double TURN_TO_WALL = 67;

    public static int TO_WALL = 900;

    public static double TURN_TO_DEPOT = 140;

    public static int TO_DEPOT = 300;

    public static int EXTEND_TO_DEPOT = 2400;

    public static int BACK_DEPOT = 250;

    public static double TURN_TO_RETURN = 62;

    public static int TO_SAMPLE = 900;

    /*
     * SAMPLING
     */
    public static double LEFT_TURN_SAMPLE = 30;
    public static double CENTER_TURN_SAMPLE = -3;
    public static double RIGHT_TURN_SAMPLE = -39;

    public static int EXTEND_TO_SAMPLE = 1200;

    public static double TURN_TO_SCORE = -2;

    // 140
    public static int BACK_TO_LANDER = 180;

    // 170
    public static int STRAFE_TO_SCORE = 250;

    public static int RAISE_TO_SCORE = 1800;

    public static int LOWER = 70;

    /*
     * PARK
     */

    public static int EXTEND_TO_CRATER = 2200;


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
            telemetry.addData("Gold Pos", robot.vision.getGoldPos());
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
            }
            telemetry.update();
        }

        waitForStart();
        goldPosition = robot.vision.getGoldPosition();
        robot.start();
        updateThread.start();

        robot.mecanumDrive.brakeMode(true);

        robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
        robot.latchingLift.runLiftToPosition(1.0, LIFT_DOWN);
        sleep(300);

        robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        robot.mecanumDrive.encoderDrive(0.8, 0, 0, STRAFE_OFF_HOOK);
        sleep(300);

        robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, TO_TAPE);
        sleep(300);

        robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, TURN_TO_WALL);
        robot.mecanumDrive.turnToAngle(TURN_SPEED_ADJUST, TURN_TO_WALL);
        sleep(300);

        robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, TO_WALL);
        sleep(300);

        robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, TURN_TO_DEPOT);
        robot.mecanumDrive.turnToAngle(TURN_SPEED_ADJUST, TURN_TO_DEPOT);
        sleep(300);

        robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, TO_DEPOT);
        sleep(300);

        robot.intake.runExtenderToPosition(-1.0, EXTEND_TO_DEPOT);

        robot.intake.dumpMarker();

        robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);

        robot.intake.retractIntakeExtender();

        robot.intake.setIntakePivotPosition(Intake.PivotPosition.UP);

        robot.mecanumDrive.encoderDrive(0, -FORWARD_SPEED, 0, BACK_DEPOT);
        sleep(300);

        robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, TURN_TO_RETURN);
        robot.mecanumDrive.turnToAngle(TURN_SPEED_ADJUST, TURN_TO_RETURN);
        sleep(300);

        robot.mecanumDrive.encoderDrive(0, -FORWARD_SPEED, 0, TO_SAMPLE);
        sleep(300);

        robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);

        switch (goldPosition) {
            case LEFT:
                robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, LEFT_TURN_SAMPLE);
                robot.mecanumDrive.turnToAngle(TURN_SPEED_ADJUST, LEFT_TURN_SAMPLE);
                break;
            default:
            case CENTER:
                robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, CENTER_TURN_SAMPLE);
                robot.mecanumDrive.turnToAngle(TURN_SPEED_ADJUST, CENTER_TURN_SAMPLE);
                break;
            case RIGHT:
                robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, RIGHT_TURN_SAMPLE);
                robot.mecanumDrive.turnToAngle(TURN_SPEED_ADJUST, RIGHT_TURN_SAMPLE);
                break;
        }

        robot.intake.setIntakePivotPosition(Intake.PivotPosition.DOWN);
        sleep(300);
        robot.intake.setIntakePower(-1.0);
        robot.intake.runExtenderToPosition(-1.0, EXTEND_TO_SAMPLE);
        sleep(900);
        robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);
        robot.intake.retractIntakeExtender();
        robot.intake.setIntakePivotPosition(Intake.PivotPosition.UP);

        sleep(900);

        robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);
        sleep(300);
        robot.intake.setIntakePower(0.0);

        robot.mecanumDrive.turnToAngle(TURN_SPEED_FAST, TURN_TO_SCORE);
        robot.mecanumDrive.turnToAngle(TURN_SPEED_ADJUST, TURN_TO_SCORE);

        robot.mecanumDrive.encoderDrive(0, -FORWARD_SPEED, 0, BACK_TO_LANDER);
        sleep(300);

        robot.mecanumDrive.encoderDrive(0.8, 0, 0, STRAFE_TO_SCORE);
        sleep(300);

        robot.outtake.runLiftToPosition(-1.0, RAISE_TO_SCORE);

        robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);

        robot.outtake.setOuttakePosition(Outtake.OuttakePosition.UP);
        sleep(1300);

//        robot.outtake.setOuttakePosition(Outtake.OuttakePosition.DOWN);
//        sleep(600);
//[]
//        robot.mecanumDrive.setBlinkinPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//
//        robot.outtake.lowerLiftToPosition(1.0, LOWER);
//        sleep(300);

        robot.intake.runExtenderToPosition(-1.0, EXTEND_TO_CRATER);

//        robot.outtake.lowerLiftToPosition(1.0, LOWER);
    }
}
