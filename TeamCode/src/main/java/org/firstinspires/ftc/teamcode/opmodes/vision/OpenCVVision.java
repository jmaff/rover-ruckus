package org.firstinspires.ftc.teamcode.opmodes.vision;

import com.ftc12835.roverruckus.vision.GoldPosition;
import com.ftc12835.roverruckus.vision.OpenCVGoldPipeline;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;

@TeleOp(name = "OpenCV Vision", group = "Vision")
public class OpenCVVision extends OpMode {
    private OpenCVGoldPipeline goldPipeline;
    @Override
    public void init() {
        goldPipeline = new OpenCVGoldPipeline();
        goldPipeline.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        goldPipeline.enable();
    }

    @Override
    public void loop() {
        telemetry.addData("x", goldPipeline.getGoldPoint().x);
        telemetry.addData("y", goldPipeline.getGoldPoint().y);
        telemetry.addData("area", goldPipeline.getGoldBoundingBox().area());

        String position;
        double goldY = goldPipeline.getGoldPoint().y;
        if (goldY < 140) {
            position = "RIGHT";
        } else if (goldY < 400) {
            position = "CENTER";
        } else {
            position = "LEFT";
        }

        if (goldY == 0) {
            position = "UNKNOWN";
        }

        telemetry.addData("Position", position);
    }

    @Override
    public void stop() {
        goldPipeline.disable();
    }
}
