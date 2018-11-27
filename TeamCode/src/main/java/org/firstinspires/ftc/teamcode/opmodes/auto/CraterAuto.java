package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Config
@Autonomous(name = "Crater Auto", group = "COMPETITION")
public class CraterAuto extends LinearOpMode {

    public static double TURN_SPEED = 0.4;
    public static double FORWARD_SPEED = -0.8;

    /*
     * DEPLOYING
     */

    public static int LIFT_DOWN = 14200;
    // strafe off hook
    public static int LEG_1 = 140;
    // move forward away from lander
    public static int LEG_2 = 250;

    /*
     * SAMPLING
     */

    // turns based on gold position
    public static int LEFT_STRAFE_SAMPLE = 250;
    public static int CENTER_STRAFE_SAMPLE = 200;
    public static int RIGHT_STRAFE_SAMPLE = 700;

    // knock off gold
    public static int LEG_3_LEFT = 0;
    public static int LEG_3_CENTER = 0;
    public static int LEG_3_RIGHT = 0;

    // knock off gold
    public static int LEG_3_LEFT_BACK = 0;
    public static int LEG_3_CENTER_BACK = 0;
    public static int LEG_3_RIGHT_BACK = 0;

    /*
     * TEAM MARKER
     */

    // turn towards field wall after sampling
    public static double TURN_WALL = 0;

    // drive to same position
    public static int LEG_4_LEFT = 0;
    public static int LEG_4_CENTER = 0;
    public static int LEG_4_RIGHT = 0;

    // turn to face depot
    public static int TURN_DEPOT = 0;

    // drive to depot to dump marker
    public static int LEG_5 = 0;

    /*
     * PARK
     */

    // return to crater and park
    public static int LEG_6 = 0;

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

        robot.mecanumDrive.encoderDrive(0.8, 0, 0, LEG_1);
        sleep(300);

        robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_2);
        sleep(300);

        switch (goldPostion) {
            case LEFT:
                robot.mecanumDrive.encoderDrive(-0.8, 0, 0, LEFT_STRAFE_SAMPLE);
                sleep(300);

                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_3_LEFT);
                sleep(300);

                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_3_LEFT_BACK);
                sleep(300);

                robot.mecanumDrive.turnToAngle(TURN_SPEED, TURN_WALL);
                sleep(300);

                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_4_LEFT);
                sleep(300);

                break;

            default:
            case CENTER:
                robot.mecanumDrive.encoderDrive(0.8, 0, 0, CENTER_STRAFE_SAMPLE);
                sleep(300);

                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_3_CENTER);
                sleep(300);

                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_3_CENTER_BACK);
                sleep(300);

                robot.mecanumDrive.turnToAngle(TURN_SPEED, TURN_WALL);
                sleep(300);

                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_4_CENTER);
                sleep(300);

                break;


            case RIGHT:
                robot.mecanumDrive.encoderDrive(0.8, 0, 0, RIGHT_STRAFE_SAMPLE);
                sleep(300);

                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_3_RIGHT);
                sleep(300);

                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_3_RIGHT_BACK);
                sleep(300);

                robot.mecanumDrive.turnToAngle(TURN_SPEED, TURN_WALL);
                sleep(300);

                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_4_RIGHT);
                sleep(300);

                break;
        }

        robot.mecanumDrive.turnToAngle(TURN_SPEED, TURN_DEPOT);
        sleep(300);

        robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_5);
        sleep(300);

        robot.intake.dumpMarker();

        robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_6);

        while (opModeIsActive()) {
            // pass to display telemetry
        }
    }
}
