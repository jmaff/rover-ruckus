package com.ftc12835.roverruckus.subsystems;

import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;

import java.util.Map;

public interface Subsystem {
    /**
     * Run control code (e.g., read sensors and update motors) and return telemetry.
     */
    Map<String, Object> update(@Nullable Canvas fieldOverlay);
}
