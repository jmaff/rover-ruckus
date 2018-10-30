package com.ftc12835.library.vision;

import android.app.Activity;
import android.graphics.Bitmap;
import android.util.Log;

import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaTrackablesImpl;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.BlockingQueue;

public class VuforiaCamera extends VuforiaLocalizerImpl {
    private VuforiaPipeline pipeline;
    private DrawViewSource displayView;

    private Thread workerThread;
    private Bitmap outputImage;
    private Bitmap bitmap;
    private Mat inputMat;
    private Mat outMat;

    private boolean enabled = false;

    public VuforiaCamera(Parameters parameters) {
        super(parameters);
    }

    public void setPipeline(VuforiaPipeline pipeline){
        this.pipeline = pipeline;
        pipeline.enable();

        displayView = pipeline.getRawView();
        setMonitorViewParent(displayView.getId());

        setFrameQueueCapacity(1);
    }

    public void enable() { enabled = true; }

    public void disable() { enabled = false; }

    public void start() {
        workerThread = new Thread(() -> {
            while(!workerThread.isInterrupted()){
                render();
            }
        });

        workerThread.setName("Vuforia Camera Thread");
        workerThread.start();

        Log.d("Vuforia Camera", workerThread.getState().toString());
    }

    private void processFrame(Frame frame){
        if(frame != null) {
            inputMat = convertFrame(frame);
            outMat = pipeline.processFrame(inputMat, null);

            if (outMat != null && !outMat.empty()) {
                bitmap = Bitmap.createBitmap(outMat.width(), outMat.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(outMat, bitmap);

                double adjustedHieght = displayView.getWidth() * outMat.height() / outMat.width();
                outputImage = Bitmap.createScaledBitmap(bitmap, displayView.getWidth(), (int) adjustedHieght, false);

                ((Activity) displayView.getContext()).runOnUiThread(() -> {
                    displayView.onFrame(outputImage);
                    displayView.invalidate();
                });

            } else {
                Log.w("Vuforia Camera", "MAT BITMAP MISMATCH OR EMPTY ERROR");
            }


        } else {
            Log.w("Vuforia Camera", "No Frame!");
        }

    }

    private void render() {
         Log.d("Vuforia Camera", "Rendering Frame");

        if(pipeline != null && enabled) {

            if(!getFrameQueue().isEmpty()){
                try {
                    processFrame(getFrameQueue().take());
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else {
                Log.w("Vuforia Camera", "Frame is empty: " + getFrameQueueCapacity());
            }
        }

    }

    public void stop(){
        close();
        ((Activity)displayView.getContext()).runOnUiThread(new Runnable() {
            @Override
            public void run() {
                workerThread.interrupt();

                pipeline.disable();
            }
        });

    }

    private Mat convertFrame(Frame frame) {
        Image rgb = null;

        // basically get the number of formats for this frame
        long numImages = frame.getNumImages();

        // set rgb object if one of the formats is RGB565
        for(int i = 0; i < numImages; i++) {
            if(frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }

        if(rgb == null) {
            Log.w("Vuforia_Camera", "Image format not found");
            return null;
        }

        // create a new bitmap and copy the byte buffer returned by rgb.getPixels() to it
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        // construct an OpenCV mat from the bitmap using Utils.bitmapToMat()
        Mat mat = new Mat(bm.getWidth(), bm.getHeight(), CvType.CV_8UC4);
        Utils.bitmapToMat(bm, mat);

        return mat;
    }


}
