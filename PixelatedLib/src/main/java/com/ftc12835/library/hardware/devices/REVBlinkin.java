package com.ftc12835.library.hardware.devices;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class REVBlinkin {
    private ServoImplEx delegate;

    public REVBlinkin(Servo delegate) {
        this.delegate = new ServoImplEx(delegate.getController(), delegate.getPortNumber());

        this.delegate.setPwmRange(new PwmControl.PwmRange(1000, 2000));
    }

    public void setRaw(double value) {
        delegate.setPosition(value);
    }

    public void setStripPattern(StripPattern pattern) {
        setRaw(pattern.getValue());
    }

    public enum StripPattern {
        RAINBOW_RAINBOW_PALETTE(1005),
        RAINBOW_PARTY_PALETTE(1015),
        RAINBOW_OCEAN_PALETTE(1025),
        RAINBOW_LAVE_PALETTE(1035),
        RAINBOW_FOREST_PALETTE(1045),
        RAINBOW_GLITTER(1055),
        CONFETTI(1065),
        SHOT_RED(1075),
        SHOT_BLUE(1085),
        SHOT_WHITE(1095),
        SINELON_RAINBOW_PALETTE(1105),
        SINELON_PARTY_PALETTE(1115),
        SINELON_OCEAN_PALETTE(1125),
        SINELON_LAVA_PALETTE(1135),
        SINELON_FOREST_PALETTE(1145),
        BPM_RAINBOW_PALETTE(1155),
        BPM_PARTY_PALETTE(1165),
        BPM_OCEAN_PALETTE(1175),
        BPM_LAVA_PALETTE(1185),
        BPM_FOREST_PALETTE(1195),
        FIRE_MEDIUM(1205),
        FIRE_LARGE(1215),
        TWINKLES_RAINBOW_PALETTE(1225),
        TWINKLES_PARTY_PALETTE(1235),
        TWINKLES_OCEAN_PALETTE(1245),
        TWINKLES_LAVA_PALETTE(1255),
        TWINKLES_FOREST_PALETTE(1265),
        COLOR_WAVES_RAINBOW_PALETTE(1275),
        COLOR_WAVES_PARTY_PALETTE(1285),
        COLOR_WAVES_OCEAN_PALETTE(1295),
        COLOR_WAVES_LAVA_PALETTE(1305),
        COLOR_WAVES_FOREST_PALETTE(1315),
        LARSON_SCANNER_RED(1325),
        LARSON_SCANNER_GRAY(1335),
        LIGHT_CHASE_RED(1345),
        LIGHT_CHASE_BLUE(1355),
        LIGHT_CHASE_GRAY(1365),
        HEARTBEAT_RED(1375),
        HEARTBEAT_BLUE(1385),
        HEARTBEAT_WHITE(1395),
        HEARTBEAT_GRAY(1405),
        BREATH_RED(1415),
        BREATH_BLUE(1425),
        BREATH_GRAY(1435),
        STROBE_RED(1445),
        STROBE_BLUE(1455),
        STROBE_GOLD(1465),
        STROBE_WHITE(1475),
        END_TO_END_BLEND_TO_BLACK_COLOR_ONE(1485),
        LARSON_SCANNER_COLOR_ONE(1495),
        LIGHT_CHASE_COLOR_ONE(1505),
        HEARTBEAT_SLOW_COLOR_ONE(1515),
        HEARTBEAT_MEDIUM_COLOR_ONE(1525),
        HEARTBEAT_FAST_COLOR_ONE(1535),
        BREATH_SLOW_COLOR_ONE(1545),
        BREATH_FAST_COLOR_ONE(1555),
        SHOT_COLOR_ONE(1565),
        STROBE_COLOR_ONE(1575),
        END_TO_END_BLEND_TO_BLACK_COLOR_TWO(1585),
        LARSON_SCANNER_COLOR_TWO(1595),
        LIGHT_CHASE_COLOR_TWO(1605),
        HEARTBEAT_SLOW_COLOR_TWO(1615),
        HEARTBEAT_MEDIUM_COLOR_TWO(1625),
        HEARTBEAT_FAST_COLOR_TWO(1635),
        BREATH_SLOW_COLOR_TWO(1645),
        BREATH_FAST_COLOR_TWO(1655),
        SHOT_COLOR_TWO(1665),
        STROBE_COLOR_TWO(1675),
        SPARKLE_ONE_ON_TWO(1685),
        SPARKLE_TWO_ON_ONE(1695),
        GRADIENT_ONE_AND_TWO(1705),
        BPM_ONE_AND_TWO(1715),
        END_TO_END_BLEND_ONE_TO_TWO(1725),
        END_TO_END_BLEND(1735),
        ONE_AND_TWO_NO_BLENDING(1745),
        TWINKLES_ONE_AND_TWO(1755),
        COLOR_WAVES_ONE_AND_TWO(1765),
        SINELONE_ONE_AND_TWO(1775),
        HOT_PINK(1785),
        DARK_RED(1795),
        RED(1805),
        RED_ORANGE(1815),
        ORANGE(1825),
        GOLD(1835),
        YELLOW(1845),
        LAWN_GREEN(1855),
        LIME(1865),
        DARK_GREEN(1875),
        GREEN(1885),
        BLUE_GREEN(1895),
        AQUA(1905),
        SKY_BLUE(1915),
        DARK_BLUE(1925),
        BLUE(1935),
        BLUE_VIOLET(1945),
        VIOLET(1955),
        WHITE(1965),
        GRAY(1975),
        DARK_GRAY(1985),
        BLACK(1995);

        private double pulseWidth;
        StripPattern(double value) { this.pulseWidth = value; }
        public double getValue() {
            return (pulseWidth - 1000) / 1000;
        }
        public int getPulseWidth() { return (int) pulseWidth; }
    }
}
