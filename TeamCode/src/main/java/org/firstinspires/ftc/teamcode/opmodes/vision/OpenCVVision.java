package org.firstinspires.ftc.teamcode.opmodes.vision;

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

        String position;
        switch (goldPipeline.getGoldPosition()) {
            case LEFT:
                position = "LEFT";
            case CENTER:
                position = "CENTER";
            case RIGHT:
                position = "RIGHT";
            default:
            case UNKNOWN:
                position = "UNKNOWN";
        }
        telemetry.addData("Position", position);
    }

    @Override
    public void stop() {
        goldPipeline.disable();
    }
}
