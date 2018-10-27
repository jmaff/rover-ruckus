package org.firstinspires.ftc.teamcode.subsystems;

import com.ftc12835.library.hardware.management.RobotTemplate;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Robot extends RobotTemplate {
    public MecanumDrive mecanumDrive;
    public LatchingLift latchingLift;

    public Robot(OpMode opMode) {
        super(opMode);
    }

    @Override
    protected void initSubsystems(OpMode opMode) {
        mecanumDrive = new MecanumDrive(opMode.hardwareMap);
        super.addSubsystem(mecanumDrive);

        super.addRevHub(mecanumDrive.frontHub);

        latchingLift = new LatchingLift(opMode.hardwareMap);
        super.addSubsystem(latchingLift);
    }
}
