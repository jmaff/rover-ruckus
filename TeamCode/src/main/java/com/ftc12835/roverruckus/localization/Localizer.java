package com.ftc12835.roverruckus.localization;

import com.acmerobotics.library.localization.Vector2d;

public interface Localizer {
    Vector2d update();
    void setEstimatedPosition(Vector2d position);
}
