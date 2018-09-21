package com.ftc12835.library.vision;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.corningrobotics.enderbots.endercv.OpenCVLoader;
import org.opencv.core.Mat;

/**
 * Created by 21maffetone on 9/20/18.
 */

public abstract class Pipeline {
    static {
        try {
            System.loadLibrary("opencv_java3");
        } catch (UnsatisfiedLinkError e) {
            OpenCVLoader.loadOpenCV();
            // pass
        }
    }
    private boolean enabled = true;

    public void internalProcessFrame(Mat frame) {
        if (enabled) {
            processFrame(frame);
        }
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public abstract void init(VisionCamera camera);
    public abstract void processFrame(Mat frame);
    public abstract void drawOverlay(CanvasOverlay overlay, int imageWidth, int imageHeight);
}
