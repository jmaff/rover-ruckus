package org.firstinspires.ftc.teamcode.test;

import com.ftc12835.library.hardware.REVBlinkin;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 21maffetone on 7/16/18.
 */

@TeleOp(name = "Blinkin Test", group = "Test")
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
