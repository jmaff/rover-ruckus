package com.ftc12835.roverruckus.subsystems;

import android.content.Context;
import android.support.annotation.IdRes;
import android.support.annotation.Nullable;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.ftc12835.library.hardware.management.Subsystem;
import com.ftc12835.library.vision.Camera;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;

public class SamplingCamera extends Camera implements Subsystem {
    private static final String VUFORIA_KEY = "AewtRPv/////AAAAGaMgsjVBM0V1iXsVvoHp5Kc2blrjOLIShY51FJwWHUQxHCUVqArnjqxr42iuWIIxZInZ4b2yZ7C+AQ5qixTVnBWu0ledKQii3pegzJlBVPUwaofmB3hWcqLJ6dObSS4Tg5tRIMKVZFmXwgVs9DrVBFXu7Jm5IHHZRVEtQOB07+fhNyiOfNAJ164Ygl/xT5AVVCt+59ksxsvUOi5IA5fymu9VSFqmFnbx5FC1ruFBmEtlEiV4e7pCuOvjqC2Di3kEaVnMcvYbKGe/vE/nivARFU0J0F5liOLtSa4Y4b1PXamJQYe6jDYJlj6W0rcB/rZH4rzs9sf6qkL1LI/Tb065BwMQ0yFvvgecl+U2fOjMGPey";

    private VuforiaLocalizer vuforia;
    private WebcamName webcam;
    private ExecutorService frameConsumerExecutor;

    public SamplingCamera(HardwareMap hardwareMap) {
        this.webcam = hardwareMap.get(WebcamName.class, "CAMERA");
    }

    private class FrameConsumer implements Runnable {
        private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;
        private Mat frame;
        private byte[] frameBuffer;

        private FrameConsumer(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue) {
            this.frameQueue = frameQueue;
        }

        @Override
        public void run() {
            while(!Thread.currentThread().isInterrupted()) {
                // grab frames and process them
                if (!frameQueue.isEmpty()) {
                    VuforiaLocalizer.CloseableFrame vuforiaFrame = null;
                    try {
                        vuforiaFrame = frameQueue.take();
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }

                    if (vuforiaFrame == null) {
                        continue;
                    }

                    for (int i = 0; i < vuforiaFrame.getNumImages(); i++) {
                        Image image = vuforiaFrame.getImage(i);
                        if (image.getFormat() == PIXEL_FORMAT.RGB888) {
                            int imageWidth = image.getWidth(), imageHeight = image.getHeight();
                            ByteBuffer byteBuffer = image.getPixels();
                            if (frameBuffer == null) {
                                frameBuffer = new byte[byteBuffer.capacity()];
                            }
                            byteBuffer.get(frameBuffer);

                            if (this.frame == null) {
                                this.frame = new Mat(imageHeight, imageWidth, CvType.CV_8UC3);
                            }
                            this.frame.put(0, 0, frameBuffer);

                            Imgproc.cvtColor(this.frame, this.frame, Imgproc.COLOR_RGB2BGR);

                            currentFrame = pipeline.process(frame);
                        }
                    }
                    vuforiaFrame.close();
                } else {
                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        }
    }

    @Override
    public void init() {
        Context context = AppUtil.getDefContext();
        @IdRes int cameraMonitorViewId;
        cameraMonitorViewId = context.getResources().getIdentifier("cameraMonitorViewId", "id", context.getPackageName());

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_KEY;
        vuforiaParams.cameraName = webcam;

        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1);

        frameConsumerExecutor = ThreadPool.newSingleThreadExecutor("Sampling camera frame consumer");
        frameConsumerExecutor.execute(new FrameConsumer(vuforia.getFrameQueue()));
    }

    public void close() {
        if (frameConsumerExecutor != null) {
            frameConsumerExecutor.shutdownNow();
            frameConsumerExecutor = null;
        }
    }

    @Override
    public Map<String, Object> update(@Nullable Canvas fieldOverlay) {
        return null;
    }


}
