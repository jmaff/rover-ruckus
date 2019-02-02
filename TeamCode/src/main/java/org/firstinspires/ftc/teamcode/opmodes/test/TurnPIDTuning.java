package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.RobotDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.ftc12835.library.control.PIDController;
import com.ftc12835.library.util.CSVWriter;
import com.ftc12835.library.util.LoggingUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.io.File;

@Config
@Autonomous(name = "Turn PID Tuning")
public class TurnPIDTuning extends LinearOpMode {
    private Robot robot;

    private Runnable updateRunnable = () -> {
        while (opModeIsActive()) {
            robot.update();
            telemetry.addData("Heading", robot.mecanumDrive.getHeading());
            telemetry.update();
        }
    };

    private Thread updateThread = new Thread(updateRunnable);

    public static double kP = 0.089;
    public static double kI = 0;
    public static double kD = 0;

    public static double setpoint = 90.0;

    private PIDController pidController;

    @Override
    public void runOpMode() {
        robot = new Robot(this, true);
        pidController = new PIDController(kP, kI, kD);
        pidController.setSetpoint(setpoint);

        Telemetry dashboardTelemetry = RobotDashboard.getInstance().getTelemetry();
        ElapsedTime timer = new ElapsedTime();

        File logRoot = LoggingUtil.getLogRoot(this);
        String prefix = "Turn_Tuning" + System.currentTimeMillis();
        CSVWriter writer = new CSVWriter(new File(logRoot, prefix + ".csv"));

        waitForStart();
        timer.reset();

        robot.start();
        updateThread.start();

        robot.mecanumDrive.brakeMode(true);

        while (opModeIsActive()) {
            robot.mecanumDrive.cartesianDrive(0, 0, -pidController.update(robot.mecanumDrive.getHeading()));
            dashboardTelemetry.addData("heading", robot.mecanumDrive.getHeading());

            writer.put("timer", timer.seconds());
            writer.put("heading", robot.mecanumDrive.getHeading());
            writer.write();
        }

        writer.close();
    }
}
