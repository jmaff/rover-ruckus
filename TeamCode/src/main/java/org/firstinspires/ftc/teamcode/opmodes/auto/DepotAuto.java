package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Config
@Autonomous(name = "Depot Auto")
public class DepotAuto extends LinearOpMode {
    public static int LIFT_DOWN = 14200;
    // strafe off hook
    public static int LEG_1 = 190;
    // move forward away from lander
    public static int LEG_2 = 70;

    public static double TURN_SPEED = -0.6;

    // turns based on gold position
    public static double LEFT_TURN_SAMPLE = 20;
    public static double CENTER_TURN_SAMPLE = 0;
    public static double RIGHT_TURN_SAMPLE = -20;

    public static int LEG_3_LEFT = 400;
    public static int LEG_3_CENTER = 900;
    public static int LEG_3_RIGHT = 400;

    // turn to face depot
    public static double LEFT_TURN_DEPOT = 0;
    public static double RIGHT_TURN_DEPOT = 0;

    // drive into depot
    public static int LEG_4_LEFT = 400;
    public static int LEG_4_CENTER = 0;
    public static int LEG_4_RIGHT = 400;

    public static double FORWARD_SPEED = -0.8;

    private Robot robot;
    private Vision.GoldPostion goldPostion;

    private Runnable updateRunnable = () -> {
        while (opModeIsActive()) {
            robot.update();
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
                robot.mecanumDrive.turnToAngle(-TURN_SPEED, LEFT_TURN_SAMPLE);
                sleep(300);
                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_3_LEFT);
                sleep(300);
                robot.mecanumDrive.turnToAngle(TURN_SPEED, LEFT_TURN_DEPOT);
                sleep(300);
                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_4_LEFT);
                break;
            default:
            case CENTER:
                robot.mecanumDrive.turnToAngle(TURN_SPEED, CENTER_TURN_SAMPLE);
                sleep(300);
                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_3_CENTER);
                break;
            case RIGHT:
                robot.mecanumDrive.turnToAngle(TURN_SPEED, RIGHT_TURN_SAMPLE);
                sleep(300);
                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_3_RIGHT);
                sleep(300);
                robot.mecanumDrive.turnToAngle(-TURN_SPEED, RIGHT_TURN_DEPOT);
                sleep(300);
                robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_4_RIGHT);
                break;
        }


        while (opModeIsActive()) {
            // pass to display telemetry
        }
    }
}
