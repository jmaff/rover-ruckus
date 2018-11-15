package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@Autonomous(name = "Depot Auto")
public class DepotAuto extends LinearOpMode {
    public static int LIFT_DOWN = 14200;
    public static int LEG_1 = 190;
    private Robot robot;

    private Runnable updateRunnable = () -> {
        while (opModeIsActive()) {
            robot.update();
        }
    };

    private Thread updateThread = new Thread(updateRunnable);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        waitForStart();
        robot.start();
        updateThread.start();
        robot.vision.enable();

        robot.latchingLift.runLiftToPosition(1.0, LIFT_DOWN);
        robot.mecanumDrive.encoderDrive(0.8, 0, 0, LEG_1);

        while (opModeIsActive()) {
            // pass to display telemetry
        }
    }
}
