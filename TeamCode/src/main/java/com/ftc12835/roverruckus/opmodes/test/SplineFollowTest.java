package com.ftc12835.roverruckus.opmodes.test;

import com.acmerobotics.library.util.CSVWriter;
import com.acmerobotics.library.util.LoggingUtil;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.ftc12835.roverruckus.roadrunner.AssetsTrajectoryLoader;
import com.ftc12835.roverruckus.subsystems.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class SplineFollowTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        Trajectory trajectory;
        try {
            trajectory = AssetsTrajectoryLoader.load("test");
        } catch (IOException e) {
            trajectory = null;
            telemetry.addData("ERROR", "Trajectory not loaded!");
        }

        File logRoot = LoggingUtil.getLogRoot(this);
        String prefix = "SplineFollowTest-" + System.currentTimeMillis();
        CSVWriter writer = new CSVWriter(new File(logRoot, prefix + ".csv"));

        waitForStart();

        double startTime = System.nanoTime() / 1e9;
        drive.getTrajectoryFollower().followTrajectory(trajectory);
        while (opModeIsActive() && drive.getTrajectoryFollower().isFollowing()) {
            double time = System.nanoTime() / 1e9;
            Pose2d currentPose = drive.getPoseEstimate();
            Pose2d targetPose = trajectory.get(time - startTime);

            writer.put("time", time - startTime);
            writer.put("targetX", targetPose.getX());
            writer.put("targetY", targetPose.getY());
            writer.put("targetHeading", targetPose.getHeading());
            writer.put("currentX", currentPose.getX());
            writer.put("currentY", currentPose.getY());
            writer.put("currentHeading", currentPose.getHeading());
            writer.write();

            drive.getTrajectoryFollower().update(currentPose);
            drive.updatePoseEstimate();
        }
    }
}
