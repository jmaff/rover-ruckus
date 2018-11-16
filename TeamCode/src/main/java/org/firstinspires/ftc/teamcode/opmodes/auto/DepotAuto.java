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
    public static int LEG_1 = 190;
    public static int LEG_2 = 70;
    public static double TURN_SPEED = -0.6;
    public static double LEFT_TURN = -20;
    public static double CENTER_TURN = -2;
    public static double RIGHT_TURN = -20;
    public static int LEG_3 = 400;
    public static double FORWARD_SPEED = -0.8;

    private Robot robot;

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

        waitForStart();
        robot.start();
        updateThread.start();

        robot.mecanumDrive.brakeMode(true);

        robot.latchingLift.runLiftToPosition(1.0, LIFT_DOWN);
        sleep(300);
        robot.mecanumDrive.encoderDrive(0.8, 0, 0, LEG_1);

        long start = System.currentTimeMillis();
        while (robot.vision.getGoldPostion() != Vision.GoldPostion.UNKNOWN && System.currentTimeMillis() < start + 3000) {
            // keep trying to find
        }

        double goldTurn;
        switch (robot.vision.getGoldPostion()) {
            case LEFT:
                goldTurn = LEFT_TURN;
                break;
            default:
            case CENTER:
                goldTurn = CENTER_TURN;
                break;
            case RIGHT:
                goldTurn = RIGHT_TURN;
                break;
        }

        robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_2);
        sleep(300);
        robot.mecanumDrive.turnToAngle(TURN_SPEED, goldTurn);
        sleep(300);
        robot.mecanumDrive.encoderDrive(0, FORWARD_SPEED, 0, LEG_3);


        while (opModeIsActive()) {
            // pass to display telemetry
        }
    }
}
