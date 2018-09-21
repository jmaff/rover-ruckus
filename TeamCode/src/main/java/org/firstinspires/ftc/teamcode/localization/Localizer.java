package org.firstinspires.ftc.teamcode.localization;

import com.ftc12835.library.localization.Vector2d;

public interface Localizer {
    Vector2d update();
    void setEstimatedPosition(Vector2d position);
}
