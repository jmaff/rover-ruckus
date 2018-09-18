package com.ftc12835.library.vision;

import org.opencv.core.Mat;

public interface Pipeline {
    Mat process(Mat frame);
}
