package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name = "Mineral Detection")
public class MineralDetection extends OpMode {

    DcMotor intake;

    DistanceSensor distance1;
    DistanceSensor distance2;

    RevBlinkinLedDriver blinkin;

    long timeBegin;

    enum IntakeStatus {
        TWO,
        ONE,
        NONE
    }

    IntakeStatus prevStatus = IntakeStatus.NONE;
    IntakeStatus status = IntakeStatus.NONE;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");

        distance1 = hardwareMap.get(DistanceSensor.class, "1");
        distance2 = hardwareMap.get(DistanceSensor.class, "2");

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }

    @Override
    public void start() {
        intake.setPower(0.8);
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    @Override
    public void loop() {
        telemetry.addData("1 Distance", distance1.getDistance(DistanceUnit.CM));
        telemetry.addData("2 Distance", distance2.getDistance(DistanceUnit.CM));

        double lower = distance1.getDistance(DistanceUnit.CM);
        double upper = distance2.getDistance(DistanceUnit.CM);

        if (lower < 5.7 && upper < 5.7) {
            telemetry.addData("Intake Status", "TWO MINERALS");
            status = IntakeStatus.TWO;
        } else if (lower < 5.7 || upper < 5.7) {
            telemetry.addData("Intake Status", "ONE MINERALS");
            status = IntakeStatus.ONE;
        } else {
            telemetry.addData("Intake Status", "NO MINERALS");
            status= IntakeStatus.NONE;
        }

        if (status != prevStatus) {
            if (status == IntakeStatus.TWO) {
                timeBegin = System.currentTimeMillis();
            } else {
                timeBegin = 0;
            }
        }

        if (timeBegin != 0 && System.currentTimeMillis() - timeBegin >= 1000) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE);
        } else {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }

        prevStatus = status;
    }
}
