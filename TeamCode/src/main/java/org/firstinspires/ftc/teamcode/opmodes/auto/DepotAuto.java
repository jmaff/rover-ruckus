package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Config
@Autonomous(name = "Depot Auto", group = "COMPETITION")
public class DepotAuto extends LinearOpMode {
    public static double TURN_SPEED = 0.4;
    public static double TURN_AROUND_SPEED = 0.4;
    public static double FORWARD_SPEED = -0.7;
    public static double X_OFFSET_SPEED = 0.2;

    /*
     * DEPLOYING
     */

    public static int LIFT_DOWN = 14200;
    // strafe off hook
    public static int STRAFE_OFF_HOOK = 125;
    // move forward away from lander
    public static int TO_TAPE = 250;

    /*
     * SAMPLING
     */

    // turns based on gold position
    public static int LEFT_STRAFE_SAMPLE = 335;
    public static int CENTER_STRAFE_SAMPLE = 200;
    public static int RIGHT_STRAFE_SAMPLE = 700;

    public static int LEFT_KNOCK_OFF = 700;
    public static int CENTER_KNOCK_OFF = 850;
    public static int RIGHT_KNOCK_OFF = 400;

    /*
     * TEAM MARKER
     */
    public static int LEFT_STRAFE_TO_WALL = 0;
    public static int CENTER_STRAFE_TO_WALL = 450;
    public static int RIGHT_STRAFE_TO_WALL = 0;

    public static int LEFT__TO_DUMP = 100;
    public static int CENTER_TO_DUMP = 150;
    public static int RIGHT_TO_DUMP = 0;

    public static double TURN_TO_DUMP = -25;

    public static int LEFT_BACK_UP = 200;
    public static int CENTER_BACK_UP = 0;
    public static int RIGHT_BACK_UP = 0;

    /*
     * PARK
     */

    public static double TURN_AROUND = 115;

    public static int LEFT_TO_CRATER = 1450;
    public static int CENTER_TO_CRATER = 1550;
    public static int RIGHT_TO_CRATER = 1550;

    private Robot robot;
    private Vision.GoldPostion goldPostion;

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
            switch (robot.vision.getGoldPostion()) {
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
        goldPostion = robot.vision.getGoldPostion();
        robot.start();
        updateThread.start();

        robot.mecanumDrive.brakeMode(true);

        robot.latchingLift.runLiftToPosition(1.0, LIFT_DOWN);
        sleep(300);

        robot.mecanumDrive.encoderDrive(0.8, 0, 0, STRAFE_OFF_HOOK);
        sleep(300);

        robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, TO_TAPE);
        sleep(300);

        switch (goldPostion) {
            case LEFT:
                robot.mecanumDrive.encoderDrive(0.8, 0, 0, LEFT_STRAFE_SAMPLE);
                sleep(300);

                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEFT_KNOCK_OFF);
                sleep(300);

                robot.mecanumDrive.encoderDrive(0.8, 0, 0, LEFT_STRAFE_TO_WALL);
                sleep(300);

                robot.mecanumDrive.turnToAngle(TURN_SPEED, TURN_TO_DUMP);

                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEFT__TO_DUMP);

                robot.intake.dumpMarker();

                robot.mecanumDrive.encoderDrive(0, -FORWARD_SPEED, 0, LEFT_BACK_UP);

                robot.mecanumDrive.turnToAngle(TURN_AROUND_SPEED, TURN_AROUND);

                robot.mecanumDrive.encoderDrive(X_OFFSET_SPEED, FORWARD_SPEED, 0, LEFT_TO_CRATER);

                robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);
                break;

            default:
            case CENTER:
                robot.mecanumDrive.encoderDrive(-0.8, 0, 0, CENTER_STRAFE_SAMPLE);
                sleep(300);

                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, CENTER_KNOCK_OFF);
                sleep(300);

                robot.mecanumDrive.encoderDrive(0.8, 0, 0, CENTER_STRAFE_TO_WALL);
                sleep(300);

                robot.mecanumDrive.turnToAngle(TURN_SPEED, TURN_TO_DUMP);

                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, CENTER_TO_DUMP);

                robot.intake.dumpMarker();

                robot.mecanumDrive.encoderDrive(0, -FORWARD_SPEED, 0, CENTER_BACK_UP);

                robot.mecanumDrive.turnToAngle(TURN_AROUND_SPEED, TURN_AROUND);

                robot.mecanumDrive.encoderDrive(X_OFFSET_SPEED, FORWARD_SPEED, 0, CENTER_TO_CRATER);

                robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);
                break;

            case RIGHT:
                robot.mecanumDrive.encoderDrive(-0.8, 0, 0, CENTER_STRAFE_SAMPLE);
                sleep(300);

                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, CENTER_KNOCK_OFF);
                sleep(300);

                robot.mecanumDrive.encoderDrive(0.8, 0, 0, CENTER_STRAFE_TO_WALL);
                sleep(300);

                robot.mecanumDrive.turnToAngle(TURN_SPEED, TURN_TO_DUMP);

                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, RIGHT_TO_DUMP);

                robot.intake.dumpMarker();

                robot.mecanumDrive.encoderDrive(0, -FORWARD_SPEED, 0, RIGHT_BACK_UP);

                robot.mecanumDrive.turnToAngle(TURN_AROUND_SPEED, TURN_AROUND);

                robot.mecanumDrive.encoderDrive(X_OFFSET_SPEED, FORWARD_SPEED, 0, RIGHT_TO_CRATER);

                robot.intake.setIntakePivotPosition(Intake.PivotPosition.MIDDLE);
                break;
        }

        while (opModeIsActive()) {
            // pass to display telemetry
        }
    }
}
