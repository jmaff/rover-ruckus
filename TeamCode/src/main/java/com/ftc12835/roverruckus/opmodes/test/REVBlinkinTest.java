package com.ftc12835.roverruckus.opmodes.test;

import com.ftc12835.library.hardware.REVBlinkin;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Blinkin Test", group = "TEST")
public class REVBlinkinTest extends OpMode {

    REVBlinkin blinkin;
    boolean prev = false;
    int index = 0;

    @Override
    public void init() {
        blinkin = new REVBlinkin(hardwareMap.get(Servo.class, "BLINKIN"));
    }

    @Override
    public void loop() {
//        if (gamepad1.x) {
//            blinkin.setStripPattern(REVBlinkin.StripPattern.TWINKLES_PARTY_PALETTE);
//        } else {
//            blinkin.setStripPattern(REVBlinkin.StripPattern.LIME);
//        }

        if (gamepad1.a != prev) {
            if (gamepad1.a) {
                index++;
            }
            prev = gamepad1.a;
        }

        blinkin.setStripPattern(REVBlinkin.StripPattern.values()[index]);

        telemetry.addData("Pulse Width", REVBlinkin.StripPattern.values()[index].getPulseWidth());

    }
}
