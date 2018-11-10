package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc12835.library.hardware.management.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake implements Subsystem {
    private DcMotor mineralLift;
    private Servo outtakePivot;

    private double liftPower;
    private double outtakePosition;

    public Outtake(HardwareMap hardwareMap) {
        mineralLift = hardwareMap.get(DcMotor.class, "MINERAL_LIFT");
        outtakePivot = hardwareMap.get(Servo.class, "OUTTAKE");
    }

    public void setLiftPower(double liftPower) {
        this.liftPower = liftPower;
    }

    public void setOuttakePosition(double outtakePosition) {
        this.outtakePosition = outtakePosition;
    }

    @Override
    public void update() {
        mineralLift.setPower(liftPower);
        outtakePivot.setPosition(outtakePosition);
    }
}
