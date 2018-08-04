package com.ftc12835.roverruckus.opmodes.vision;

import android.app.Activity;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Log;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.ImageButton;
import android.widget.LinearLayout;

import com.acmerobotics.dashboard.RobotDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.ftc12835.roverruckus.VisionConstants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CountDownLatch;

@Disabled
@TeleOp(name = "JewelVision", group = "VISION")
public class JewelVision extends LinearOpMode {
    public static final float JEWEL_PLATFORM_ASPECT_RATIO = 2.6f; // width/height
    public static final double COLOR_THRESHOLD = 0.7;

    public static final String TAG = "JewelVision";

    private Mat raw, rgb;
    private byte[] imgData;
    private CountDownLatch openCvInitialized = new CountDownLatch(1);
    VuforiaLocalizer vuforia;
    BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;
    private int imageWidth, imageHeight;
    private Rect jewelPlatformRect, leftJewelRect, rightJewelRect;
    private double leftRed, leftBlue, rightRed, rightBlue;
    private FrameLayout parentLayout;

    private static void drawOpenCvRect(Canvas canvas, Rect rect, Paint paint) {
        canvas.drawRect(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height, paint);
    }

    @Override public void runOpMode() throws InterruptedException {

        final Paint paint = new Paint();
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(5.0f);

        final View overlayView = new View(hardwareMap.appContext) {
            @Override
            protected void onDraw(Canvas canvas) {
                super.onDraw(canvas);

                // compute the portion of the canvas that overlaps the camera preview
                int canvasWidth = canvas.getWidth(), canvasHeight = canvas.getHeight();
                float canvasAspectRatio = (float) canvasWidth / canvasHeight;

                float imageAspectRatio = (float) imageHeight / imageWidth;

                int overlayWidth, overlayHeight;
                if (canvasAspectRatio > imageAspectRatio) {
                    // width is bigger
                    overlayWidth = canvasHeight;
                    overlayHeight = (int) (canvasHeight * imageAspectRatio);
                } else {
                    // height is bigger
                    overlayHeight = canvasWidth;
                    overlayWidth = (int) (canvasWidth / imageAspectRatio);
                }

                // transform the canvas so that image coordinates can be used
                canvas.translate(canvasWidth / 2.0f, canvasHeight / 2.0f);
                canvas.rotate(90.0f);
                canvas.scale((float) overlayHeight / imageHeight, (float) overlayWidth / imageWidth);
                canvas.translate(-imageWidth / 2.0f, -imageHeight / 2.0f);

                // draw the jewel rects
                if (jewelPlatformRect != null) {
                    paint.setColor(Color.GREEN);
                    drawOpenCvRect(canvas, jewelPlatformRect, paint);

                    if (leftBlue > COLOR_THRESHOLD) {
                        paint.setColor(Color.BLUE);
                    } else if (leftRed > COLOR_THRESHOLD) {
                        paint.setColor(Color.RED);
                    } else {
                        paint.setColor(Color.GREEN);
                    }
                    drawOpenCvRect(canvas, leftJewelRect, paint);

                    if (rightBlue > COLOR_THRESHOLD) {
                        paint.setColor(Color.BLUE);
                    } else if (rightRed > COLOR_THRESHOLD) {
                        paint.setColor(Color.RED);
                    } else {
                        paint.setColor(Color.GREEN);
                    }
                    drawOpenCvRect(canvas, rightJewelRect, paint);
                }
            }
        };

        final BaseLoaderCallback loaderCallback = new BaseLoaderCallback(hardwareMap.appContext) {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS:
                    {
                        Log.i(TAG, "OpenCV loaded successfully");
                        openCvInitialized.countDown();
                    } break;
                    default:
                    {
                        super.onManagerConnected(status);
                    } break;
                }
            }
        };

        AppUtil.getInstance().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION, hardwareMap.appContext, loaderCallback);
            }
        });

        this.vuforia = ClassFactory.createVuforiaLocalizer(VisionConstants.VUFORIA_PARAMETERS);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(10);

        frameQueue = vuforia.getFrameQueue();

        final Activity activity = AppUtil.getInstance().getActivity();
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                LinearLayout cameraMonitorView = (LinearLayout) activity.findViewById(R.id.cameraMonitorViewId);
                parentLayout = (FrameLayout) cameraMonitorView.getParent();
                parentLayout.addView(overlayView);
            }
        });

        openCvInitialized.await();

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // grab frames and process them
            if (!frameQueue.isEmpty()) {
                VuforiaLocalizer.CloseableFrame frame = frameQueue.take();
                for (int i = 0; i < frame.getNumImages(); i++) {
                    Image image = frame.getImage(i);
                    if (image.getFormat() == PIXEL_FORMAT.RGB888) {
                        imageWidth = image.getWidth();
                        imageHeight = image.getHeight();
                        ByteBuffer byteBuffer = image.getPixels();
                        if (imgData == null || imgData.length != byteBuffer.capacity()) {
                            imgData = new byte[byteBuffer.capacity()];
                        }
                        byteBuffer.get(imgData);
                        if (raw == null || raw.width() != imageWidth || raw.height() != imageHeight) {
                            raw = new Mat(imageHeight, imageWidth, CvType.CV_8UC3);
                            rgb = new Mat();

                            int platformWidth = imageWidth / 2;
                            int platformHeight = (int) (platformWidth / JEWEL_PLATFORM_ASPECT_RATIO);
                            int offsetX = (imageWidth - platformWidth) / 2;
                            int offsetY = (imageHeight - platformHeight) / 2;

                            jewelPlatformRect = new Rect(offsetX, offsetY, platformWidth, platformHeight);
                            leftJewelRect = new Rect(offsetX, offsetY, platformHeight, platformHeight);
                            rightJewelRect = new Rect(offsetX + platformWidth - platformHeight, offsetY, platformHeight, platformHeight);
                        }
                        raw.put(0, 0, imgData);
                        Imgproc.cvtColor(raw, rgb, Imgproc.COLOR_RGB2BGR);

                        Mat leftJewelCropped = rgb.submat(leftJewelRect);
                        Mat rightJewelCropped = rgb.submat(rightJewelRect);

                        Scalar leftJewelTotalColor = Core.sumElems(leftJewelCropped);
                        Scalar rightJewelTotalColor = Core.sumElems(rightJewelCropped);

                        leftBlue = leftJewelTotalColor.val[0];
                        leftRed = leftJewelTotalColor.val[2];
                        rightBlue = rightJewelTotalColor.val[0];
                        rightRed = rightJewelTotalColor.val[2];

                        double leftTotal = leftBlue + leftRed;
                        double rightTotal = rightBlue + rightRed;

                        if (leftTotal == 0) {
                            leftBlue = 0;
                            leftRed = 0;
                        } else {
                            leftBlue /= leftTotal;
                            leftRed /= leftTotal;
                        }

                        if (rightTotal == 0) {
                            rightBlue = 0;
                            rightRed = 0;
                        } else {
                            rightBlue /= rightTotal;
                            rightRed /= rightTotal;
                        }

                        // make sure to release mats afterward; memory leaks are bad and annoying
                        rgb.release();
                        raw.release();
                    }
                }
                frame.close();
            }

            telemetry.addData("red", "%.2f / %.2f", leftRed, rightRed);
            telemetry.addData("blue", "%.2f / %.2f", leftBlue, rightBlue);

            telemetry.update();

            overlayView.postInvalidate();
        }

        parentLayout.removeView(overlayView);
    }
}
