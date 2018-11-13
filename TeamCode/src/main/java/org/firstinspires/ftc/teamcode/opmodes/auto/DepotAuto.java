package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class DepotAuto extends LinearOpMode {
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

        // do auto stuff here

        // drive forward 200 counts (probably wont work lol)
        robot.mecanumDrive.encoderDrive(0, 1, 0, 200);
    }
}
