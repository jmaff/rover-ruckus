package org.firstinspires.ftc.teamcode;

import com.ftc12835.pixelatedlib.hardware.REVBlinkin;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Blinkin Test", group = "TEST")
public class REVBlinkinTest extends OpMode {
    REVBlinkin blinkin;

    @Override
    public void init() {
        blinkin = new REVBlinkin(hardwareMap.get(Servo.class, "BLINKIN"));
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            blinkin.setStripPattern(REVBlinkin.StripPattern.TWINKLES_PARTY_PALETTE);
        } else {
            blinkin.setStripPattern(REVBlinkin.StripPattern.LIME);
        }
    }
}
