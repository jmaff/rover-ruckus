package com.ftc12835.library.vision;

import android.app.Activity;
import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

import java.util.concurrent.CountDownLatch;


public abstract class Camera {
    public static final String TAG = "Camera";

    private boolean initialized;

    protected AppUtil appUtil;
    protected Activity activity;
    protected Pipeline pipeline;
    protected Mat currentFrame;

    public Camera() {
        this.appUtil = AppUtil.getInstance();
        this.activity = appUtil.getActivity();
    }

    public abstract void init();

    public void attachPipeline(Pipeline pipeline) {
        this.pipeline = pipeline;
    }

    public Mat getCurrentFrame() {
        return currentFrame;
    }


}
