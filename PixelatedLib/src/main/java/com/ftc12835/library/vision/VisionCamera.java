package com.ftc12835.library.vision;

import android.app.Activity;
import android.content.Context;
import android.support.annotation.IdRes;
import android.widget.FrameLayout;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.corningrobotics.enderbots.endercv.OpenCVLoader;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;

public class VisionCamera {
    static {
        try {
            System.loadLibrary("opencv_java3");
        } catch (UnsatisfiedLinkError e) {
            OpenCVLoader.loadOpenCV();
            // pass
        }
    }

    public static class Parameters {
        @IdRes public int cameraMonitorViewId;
        public VuforiaLocalizer.CameraDirection cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        public Parameters() {
            Context context = AppUtil.getDefContext();
            cameraMonitorViewId = context.getResources().getIdentifier("cameraMonitorViewId", "id", context.getPackageName());
        }
    }

    private final List<Pipeline> pipelines;
    private AppUtil appUtil = AppUtil.getInstance();
    private Activity activity;

    private Parameters parameters;

    private boolean initialized;

    public VisionCamera(Parameters parameters) {
        this.parameters = parameters;
        this.activity = appUtil.getActivity();
        this.pipelines = new ArrayList<>();
    }

    public VisionCamera() {
        this(new Parameters());
    }

    private static final String VUFORIA_KEY = "AewtRPv/////AAAAGaMgsjVBM0V1iXsVvoHp5Kc2blrjOLIShY51FJwWHUQxHCUVqArnjqxr42iuWIIxZInZ4b2yZ7C+AQ5qixTVnBWu0ledKQii3pegzJlBVPUwaofmB3hWcqLJ6dObSS4Tg5tRIMKVZFmXwgVs9DrVBFXu7Jm5IHHZRVEtQOB07+fhNyiOfNAJ164Ygl/xT5AVVCt+59ksxsvUOi5IA5fymu9VSFqmFnbx5FC1ruFBmEtlEiV4e7pCuOvjqC2Di3kEaVnMcvYbKGe/vE/nivARFU0J0F5liOLtSa4Y4b1PXamJQYe6jDYJlj6W0rcB/rZH4rzs9sf6qkL1LI/Tb065BwMQ0yFvvgecl+U2fOjMGPey";

    private VuforiaLocalizer vuforia;
    private FrameLayout cameraLayout;
    private OverlayView overlayView;
    private ExecutorService frameConsumerExecutor;

    public void init() {
        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(parameters.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_KEY;
        vuforiaParams.cameraDirection = parameters.cameraDirection;
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1);

        if (parameters.cameraMonitorViewId != 0) {
            this.overlayView = new OverlayView(activity);

            for (Pipeline pipeline : pipelines) {
                overlayView.attachPipeline(pipeline);
            }

            final Activity activity = appUtil.getActivity();
            activity.runOnUiThread(() -> {
                LinearLayout cameraMonitorView = activity.findViewById(parameters.cameraMonitorViewId);
                cameraLayout = (FrameLayout) cameraMonitorView.getParent();
                cameraLayout.addView(overlayView);
            });
        }

        frameConsumerExecutor = ThreadPool.newSingleThreadExecutor("Vision camera frame consumer");
        frameConsumerExecutor.execute(new FrameConsumer(vuforia.getFrameQueue()));

        synchronized (pipelines) {
            for (Pipeline tracker : pipelines) {
                tracker.init(this);
            }

            initialized = true;
        }
    }

    public void addPipeline(Pipeline pipeline) {
        synchronized (pipelines) {
            this.pipelines.add(pipeline);
        }

        if (initialized) {
            pipeline.init(this);
        }

        if (overlayView != null) {
            this.overlayView.attachPipeline(pipeline);
        }
    }

    public List<Pipeline> getPipelines() {
        return pipelines;
    }

    public void onFrame(Mat frame) {
        synchronized (pipelines) {
            for (Pipeline pipeline : pipelines) {
                pipeline.internalProcessFrame(frame);
            }
        }

        if (overlayView != null) {
            overlayView.postInvalidate();
        }
    }

    public void close() {
        if (overlayView != null) {
            appUtil.runOnUiThread(() -> {
                cameraLayout.removeView(overlayView);
                overlayView = null;
            });
        }

        if (frameConsumerExecutor != null) {
            frameConsumerExecutor.shutdownNow();
            frameConsumerExecutor = null;
        }
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
                                if (overlayView != null) {
                                    overlayView.setImageSize(imageWidth, imageHeight);
                                }
                            }
                            this.frame.put(0, 0, frameBuffer);

                            Imgproc.cvtColor(this.frame, this.frame, Imgproc.COLOR_RGB2BGR);

                            onFrame(this.frame);
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
}
