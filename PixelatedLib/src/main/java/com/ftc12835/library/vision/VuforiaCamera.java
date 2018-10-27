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
    VuforiaPipeline pipeline;
    DrawViewSource displayView;
    boolean dogeCVEnabled;
    boolean showDebug = false;

    Thread workerThread;
    Bitmap outputImage;
    Bitmap bitmap;
    Mat inputMat;
    Mat outMat;
    BlockingQueue<CloseableFrame> frames;

    public VuforiaCamera(Parameters parameters) {
        super(parameters);
    }

    public void setDogeCVDetector(VuforiaPipeline pipeline){
        this.pipeline = pipeline;
        pipeline.enable();
        displayView = pipeline.getRawView();
        setMonitorViewParent(displayView.getId());

        setFrameQueueCapacity(1);
    }

    public void start(){
        workerThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while(!workerThread.isInterrupted()){
                    render();
                }
            }
        });

        workerThread.setName("Dogeforia Thread");
        workerThread.start();

        Log.d("DogeCV", workerThread.getState().toString());
    }

    public void enableDogeCV(){

        dogeCVEnabled = true;
    }

    public void disableDogeCV(){
        dogeCVEnabled = false;
    }
    public void enableTrack(){
        startTracker();
    }

    public void disableTrack() {
        stopTracker();
    }
    public void showDebug(){
        showDebug = true;

    }

    public void processFrame(Frame frame){
        if(frame != null) {
//            inputMat = new Mat(bitmap.getWidth(), bitmap.getHeight(), CvType.CV_8UC1);
//            Utils.bitmapToMat(bitmap, inputMat);

            inputMat = convertFrame(frame);

            outMat = pipeline.processFrame(inputMat, null);

            if (outMat != null && !outMat.empty()) {

                bitmap = Bitmap.createBitmap(outMat.width(), outMat.height(), Bitmap.Config.RGB_565);
                Utils.matToBitmap(outMat, bitmap);


                //height = <user-chosen width> * original height / original width
                double adjustedHieght = displayView.getWidth() * outMat.height() / outMat.width();
                outputImage = Bitmap.createScaledBitmap(bitmap, displayView.getWidth(), (int) adjustedHieght, false);

                ((Activity) displayView.getContext()).runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        displayView.onFrame(outputImage);
                        displayView.invalidate();
                    }
                });

            } else {
                Log.w("DogeCV", "MAT BITMAP MISMATCH OR EMPTY ERROR");
            }


        } else {
            Log.d("DogeCV", "No Frame!");
        }

    }

    public void render() {
         Log.d("DogeCV", "Rendering Frame");
        // super.onRenderFrame()

        if(pipeline != null && dogeCVEnabled){

            if(!getFrameQueue().isEmpty()){
                try {
                    processFrame(getFrameQueue().take());
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            } else {
                Log.w("DogeCV", "Frame is empty: " + getFrameQueueCapacity());
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
        Log.d("Vuforia_Camera", "num images: " + numImages);

        // set rgb object if one of the formats is RGB565
        for(int i = 0; i < numImages; i++) {
            Log.d("Vuforia_Camera", "pixel format: " + frame.getImage(i).getFormat());
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

        // convert to BGR before returning
//        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2BGR);

        return mat;
    }


}
