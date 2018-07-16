package com.ftc12835.pixelatedlib.hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

public class REVBlinkin {
    private ServoImplEx device;

    public REVBlinkin(Servo rawDevice) {
        device = new ServoImplEx(rawDevice.getController(), rawDevice.getPortNumber());

        device.setPwmRange(new PwmControl.PwmRange(1000, 2000));
    }

    public void setRaw(double value) {
        device.setPosition(value);
    }

    public void setStripPattern(StripPattern pattern) {
        setRaw(pattern.getValue());
    }

    public enum StripPattern {
        RAINBOW_RAINBOW_PALETTE(-0.99),
        RAINBOW_PARTY_PALETTE(-0.97),
        RAINBOW_OCEAN_PALETTE(-0.95),
        RAINBOW_LAVE_PALETTE(-0.93),
        RAINBOW_FOREST_PALETTE(-0.91),
        RAINBOW_GLITTER(-0.89),
        CONFETTI(-0.87),
        SHOT_RED(-0.85),
        SHOT_BLUE(-0.83),
        SHOT_WHITE(-0.81),
        SINELON_RAINBOW_PALETTE(-0.79),
        SINELON_PARTY_PALETTE(-0.77),
        SINELON_OCEAN_PALETTE(-0.75),
        SINELON_LAVA_PALETTE(-0.73),
        SINELON_FOREST_PALETTE(-0.71),
        BPM_RAINBOW_PALETTE(-0.69),
        BPM_PARTY_PALETTE(-0.67),
        BPM_OCEAN_PALETTE(-0.65),
        BPM_LAVA_PALETTE(-0.63),
        BPM_FOREST_PALETTE(-0.61),
        FIRE_MEDIUM(-0.59),
        FIRE_LARGE(-0.57),
        TWINKLES_RAINBOW_PALETTE(-0.55),
        TWINKLES_PARTY_PALETTE(-0.53),
        TWINKLES_OCEAN_PALETTE(-0.51),
        TWINKLES_LAVA_PALETTE(-0.49),
        TWINKLES_FOREST_PALETTE(-0.47),
        COLOR_WAVES_RAINBOW_PALETTE(-0.45),
        COLOR_WAVES_PARTY_PALETTE(-0.43),
        COLOR_WAVES_OCEAN_PALETTE(-0.41),
        COLOR_WAVES_LAVA_PALETTE(-0.39),
        COLOR_WAVES_FOREST_PALETTE(-0.37),
        LARSON_SCANNER_RED(-0.35),
        LARSON_SCANNER_GRAY(-0.33),
        LIGHT_CHASE_RED(-0.31),
        LIGHT_CHASE_BLUE(-0.29),
        LIGHT_CHASE_GRAY(-0.27),
        HEARTBEAT_RED(-0.25),
        HEARTBEAT_BLUE(-0.23),
        HEARTBEAT_WHITE(-0.21),
        HEARTBEAT_GRAY(-0.19),
        BREATH_RED(-0.17),
        BREATH_BLUE(-0.15),
        BREATH_GRAY(-0.13),
        STROBE_RED(-0.11),
        STROBE_BLUE(-0.09),
        STROBE_GOLD(-0.07),
        STROBE_WHITE(-0.05),
        END_TO_END_BLEND_TO_BLACK_COLOR_ONE(-0.03),
        LARSON_SCANNER_COLOR_ONE(-0.01),
        LIGHT_CHASE_COLOR_ONE(0.01),
        HEARTBEAT_SLOW_COLOR_ONE(0.03),
        HEARTBEAT_MEDIUM_COLOR_ONE(0.05),
        HEARTBEAT_FAST_COLOR_ONE(0.07),
        BREATH_SLOW_COLOR_ONE(0.09),
        BREATH_FAST_COLOR_ONE(0.11),
        SHOT_COLOR_ONE(0.13),
        STROBE_COLOR_ONE(0.15),
        END_TO_END_BLEND_TO_BLACK_COLOR_TWO(0.17),
        LARSON_SCANNER_COLOR_TWO(0.19),
        LIGHT_CHASE_COLOR_TWO(0.21),
        HEARTBEAT_SLOW_COLOR_TWO(0.23),
        HEARTBEAT_MEDIUM_COLOR_TWO(0.25),
        HEARTBEAT_FAST_COLOR_TWO(0.27),
        BREATH_SLOW_COLOR_TWO(0.29),
        BREATH_FAST_COLOR_TWO(0.31),
        SHOT_COLOR_TWO(0.33),
        STROBE_COLOR_TWO(0.35),
        SPARKLE_ONE_ON_TWO(0.37),
        SPARKLE_TWO_ON_ONE(0.39),
        GRADIENT_ONE_AND_TWO(0.41),
        BPM_ONE_AND_TWO(0.43),
        END_TO_END_BLEND_ONE_TO_TWO(0.45),
        END_TO_END_BLEND(0.47),
        ONE_AND_TWO_NO_BLENDING(0.49),
        TWINKLES_ONE_AND_TWO(0.51),
        COLOR_WAVES_ONE_AND_TWO(0.53),
        SINELONE_ONE_AND_TWO(0.55),
        HOT_PINK(0.57),
        DARK_RED(0.59),
        RED(0.61),
        RED_ORANGE(0.63),
        ORANGE(0.65),
        GOLD(0.67),
        YELLOW(0.69),
        LAWN_GREEN(0.71),
        LIME(0.73),
        DARK_GREEN(0.75),
        GREEN(0.77),
        BLUE_GREEN(0.79),
        AQUA(0.81),
        SKY_BLUE(0.83),
        DARK_BLUE(0.85),
        BLUE(0.87),
        BLUE_VIOLET(0.89),
        VIOLET(0.91),
        WHITE(0.93),
        GRAY(0.95),
        DARK_GRAY(0.97),
        BLACK(0.99);

        private double value;
        StripPattern(double value) { this.value = value; }
        public double getValue() { return value; }
    }
}
