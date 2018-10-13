package com.ftc12835.library.vision;

import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.AsyncTask;
import android.support.annotation.IdRes;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.ImageView;
import android.widget.LinearLayout;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ThreadPool;
import com.qualcomm.robotcore.util.WeakReferenceSet;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.corningrobotics.enderbots.endercv.OpenCVLoader;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.lang.ref.WeakReference;
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

    private static final String VUFORIA_KEY = "AfqJicT/////AAABmV0Cv8mz10n/joVc93wk6uxXh5eJuApDAXck9Oc1MWCIiRkjrE47UP6ODjidREdy5b3zc1HGJT3/B82inSlLRCa4WpjzbZvHl5FOslDg1EfWtHJH7BhXW8hVVXZJvSSwwmEP8jRWXFudvIMF5QxvVUxcj60RaL1amo00qbROMK5OOJrfDIj4Knbox6pJX17d0WDwJ66XqPbtCQstlPyHSr/iGH81fxLa0cr2Gvk5YiWHNUI4drOz7yPvFzR5Ja7J+MUagIbqtCWmWeUjgEcKqIZw0C4rfJbBsVAJ+Zx58q96hng3GgbXqZLJMcnK1kqOgNXm874Mm+XUQLMWsIIhwBMoMzg0hp5jlDxI0+2XDYGL";

    private VuforiaLocalizer vuforia;
    private FrameLayout cameraLayout;
    private OverlayView overlayView;
    private ImageView imageView;
    private ExecutorService frameConsumerExecutor;

    BitmapWorkerTask bitmapTask;

    public void init(HardwareMap hardwareMap) {
        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters();
        vuforiaParams.vuforiaLicenseKey = VUFORIA_KEY;
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "SAMPLING_CAMERA");
        vuforiaParams.cameraName = webcamName;
        vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1);

        if (parameters.cameraMonitorViewId != 0) {
            this.overlayView = new OverlayView(activity);
            this.imageView = new ImageView(activity);

            for (Pipeline pipeline : pipelines) {
                overlayView.attachPipeline(pipeline);
            }

            final Activity activity = appUtil.getActivity();

            activity.runOnUiThread(() -> {
                LinearLayout cameraMonitorView = activity.findViewById(parameters.cameraMonitorViewId);
                cameraLayout = (FrameLayout) cameraMonitorView.getParent();
//                cameraLayout.addView(overlayView);

                cameraLayout.addView(imageView);

                bitmapTask = new BitmapWorkerTask(imageView);
                bitmapTask.execute(imageView.getId());
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
        Mat m = Mat.zeros(100, 400, CvType.CV_8UC3);
        Imgproc.putText(m, "oof", new Point(30,80), Core.FONT_HERSHEY_SIMPLEX, 2.2, new Scalar(200, 200, 0), 2);

        bitmapTask.setCurrentFrame(m);
        bitmapTask.execute();

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

    private class BitmapWorkerTask extends AsyncTask<Integer, Void, Bitmap> {
        private final WeakReference<ImageView> imageViewReference;
        private int data = 0;
        private Mat currentFrame = new Mat(1080, 1920, CvType.CV_8UC3);

        public BitmapWorkerTask(ImageView imageView) {
            imageViewReference = new WeakReference<ImageView>(imageView);
        }

        public void setCurrentFrame(Mat frame) {
            currentFrame = frame;
        }

        @Override
        protected Bitmap doInBackground(Integer... integers) {
            Bitmap bm = Bitmap.createBitmap(currentFrame.cols(), currentFrame.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(currentFrame, bm);
            return bm;
        }

        @Override
        protected void onPostExecute(Bitmap bitmap) {
            if (imageViewReference != null && bitmap != null) {
                final ImageView imageView = imageViewReference.get();
                if (imageView != null) {
                    imageView.setImageBitmap(bitmap);
                }
            }
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
