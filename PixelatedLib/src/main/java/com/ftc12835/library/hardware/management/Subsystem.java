package com.ftc12835.library.hardware.management;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;

import java.util.Map;

public interface Subsystem {
    /**
     * Run control code (e.g., read sensors and update motors) and return telemetry.
     */
    void update();
}
