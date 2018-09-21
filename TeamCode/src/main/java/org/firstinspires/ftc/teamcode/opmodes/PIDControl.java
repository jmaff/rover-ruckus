package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.RobotDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.ftc12835.library.control.DerivativeController;
import com.ftc12835.library.control.IntegralController;
import com.ftc12835.library.control.ProportionalController;
import com.ftc12835.library.localization.Angle;
import com.ftc12835.library.util.CSVWriter;
import com.ftc12835.library.util.LoggingUtil;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;

@Autonomous(name = "PID Control", group = "PID")
@Config
public class PIDControl extends LinearOpMode{
    public static double kP = 0.04;
    public static double kI = 0.0;
    public static double kD = 0.07;

    public static ProportionalController pController;
    public static IntegralController iController;
    public static DerivativeController dController;

    private ElapsedTime timer = new ElapsedTime();
    private RobotDashboard dashboard;
    private BNO055IMU imu;
    private DcMotor left1;
    private DcMotor left2;
    private DcMotor right1;
    private DcMotor right2;
    private DcMotor center;
    private Servo arm;

    private double currentAngle;
    private double error;
    private double output;
    private double lastHeading = 0;
    private double integratedZAxis = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        left1 = hardwareMap.get(DcMotor.class,  "L1");
        left2 = hardwareMap.get(DcMotor.class, "L2");
        right1 = hardwareMap.get(DcMotor.class, "R1");
        right2 = hardwareMap.get(DcMotor.class, "R2");
        center = hardwareMap.get(DcMotor.class, "C");
        arm = hardwareMap.get(Servo.class, "ARM");

        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        dashboard = RobotDashboard.getInstance();
        dashboard.updateConfig();

        Telemetry dashboardTelemetry = RobotDashboard.getInstance().getTelemetry();

        File logRoot = LoggingUtil.getLogRoot(this);
        String prefix = "DerivativeControl" + System.currentTimeMillis();
        CSVWriter writer = new CSVWriter(new File(logRoot, prefix + ".csv"));

       pController = new ProportionalController(kP);
       iController = new IntegralController(kI);
       dController = new DerivativeController(kD);

        waitForStart();
        timer.reset();

        do {
            currentAngle = getIntegratedZAxis();
            error = currentAngle - 90.0;
            output = pController.update(error) + iController.update(error) + dController.update(error);

            left1.setPower(-output);
            left2.setPower(output);
            right1.setPower(-output);
            right2.setPower(output);

            writer.put("timer", timer.seconds());
            writer.put("error", error);
            writer.put("output", output);
            writer.put("heading", currentAngle);
            writer.write();

            dashboardTelemetry.addData("time", timer.seconds());
            dashboardTelemetry.addData("error", error);
            dashboardTelemetry.addData("output", output);
            dashboardTelemetry.addData("heading", currentAngle);
            dashboardTelemetry.update();
        } while (opModeIsActive());

        left1.setPower(0);
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);
        writer.close();
    }

    double getIntegratedZAxis() {
        double newHeading = imu.getAngularOrientation().firstAngle;
        double deltaHeading = newHeading - lastHeading;
        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if(deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        integratedZAxis += deltaHeading;
        lastHeading = newHeading;
        return integratedZAxis;
    }
}
