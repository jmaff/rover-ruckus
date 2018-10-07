package com.ftc12835.roverruckus.vision;

import com.acmerobotics.dashboard.config.Config;
import com.ftc12835.library.vision.ColorBlobDetector;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgproc.Imgproc;
import org.opencv.osgi.OpenCVInterface;

import java.util.List;

@Config
public class OpenCVGoldPipeline extends OpenCVPipeline {
    public static double LEFT_MIN_THRESHOLD = 400;
    public static double RIGHT_MAX_THRESHOLD = 100;

    private Mat blurOutput = new Mat();
    private SamplingPipeline.BlurType blurType = SamplingPipeline.BlurType.get("Gaussian Blur");
    private double blurRadius = 7.547169811320756;

    public static double hueMin = 80;
    public static double hueMax = 110;

    public static double satMin = 200;
    public static double satMax = 255;

    public static double valueMin = 100;
    public static double valueMax = 255;

    private Mat hsvThresholdOutput = new Mat();

    private Mat cvErodeOutput = new Mat();
    private Mat cvErodeKernel = new Mat();
    private Point cvErodeAnchor = new Point(-1, -1);
    private double cvErodeIterations = 1.0;
    private int cvErodeBordertype = Core.BORDER_CONSTANT;
    private Scalar cvErodeBordervalue = new Scalar(-1);

    private Mat maskOutput = new Mat();

    private double findBlobsMinArea = 1.0;
    private double[] findBlobsCircularity = {0.0, 1.0};
    private boolean findBlobsDarkBlobs = false;

    private List<MatOfPoint> findBlobsOutput;

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        // Step Blur0:
        blur(rgba, blurType, blurRadius, blurOutput);

        double[] hsvThresholdHue = {hueMin, hueMax};
        double[] hsvThresholdSaturation = {satMin, satMax};
        double[] hsvThresholdValue = {valueMin, valueMax};
        // Step HSV_Threshold0:
        hsvThreshold(blurOutput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

        // Step CV_erode0:
        cvErode(hsvThresholdOutput, cvErodeKernel, cvErodeAnchor, cvErodeIterations, cvErodeBordertype, cvErodeBordervalue, cvErodeOutput);

        // Step Mask0:
        mask(blurOutput, cvErodeOutput, maskOutput);

        // Step Find_Blobs0:
        findBlobsOutput = findBlobs(maskOutput, findBlobsMinArea, findBlobsCircularity, findBlobsDarkBlobs);

        for (MatOfPoint matOfPoint : findBlobsOutput) {
            Rect box = Imgproc.boundingRect(matOfPoint);
            Imgproc.rectangle(rgba, box.tl(), box.br(), new Scalar(0,255,0));
        }

        return rgba;
    }

    public Point getGoldPoint() {
        if (!findBlobsOutput.isEmpty()) {
            Rect box = Imgproc.boundingRect(findBlobsOutput.get(0));
            return new Point(box.x, box.y);
        } else {
            return new Point(0, 0);
        }
    }

    public Rect getGoldBoundingBox() {
        if (!findBlobsOutput.isEmpty()) {
            return Imgproc.boundingRect(findBlobsOutput.get(0));
        } else {
            return new Rect(0, 0, 0, 0);
        }
    }

    public GoldPosition getGoldPosition() {
        double goldY = getGoldPoint().y;

        if (goldY < RIGHT_MAX_THRESHOLD) {
            return GoldPosition.RIGHT;
        } else if (goldY < LEFT_MIN_THRESHOLD) {
            return GoldPosition.CENTER;
        } else {
            return GoldPosition.LEFT;
        }
    }

    /**
     * Code used for CV_flip.
     * Per OpenCV spec 0 -> flip on X axis.
     * >0 -> flip on Y axis.
     * <0 -> flip on both axes.
     */
    public enum FlipCode {
        X_AXIS(0),
        Y_AXIS(1),
        BOTH_AXES(-1);
        public final int value;
        FlipCode(int value) {
            this.value = value;
        }
    }

    /**
     * Flips an image along X, Y or both axes.
     * @param src Image to flip.
     * @param flipcode FlipCode of which direction to flip.
     * @param dst flipped version of the Image.
     */
    private void cvFlip(Mat src, SamplingPipeline.FlipCode flipcode, Mat dst) {
        Core.flip(src, dst, flipcode.value);
    }

    /**
     * Scales and image to an exact size.
     * @param input The image on which to perform the Resize.
     * @param width The width of the output in pixels.
     * @param height The height of the output in pixels.
     * @param interpolation The type of interpolation.
     * @param output The image in which to store the output.
     */
    private void resizeImage(Mat input, double width, double height,
                             int interpolation, Mat output) {
        Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
    }

    /**
     * An indication of which type of filter to use for a blur.
     * Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
     */
    enum BlurType{
        BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
        BILATERAL("Bilateral Filter");

        private final String label;

        BlurType(String label) {
            this.label = label;
        }

        public static BlurType get(String type) {
            if (BILATERAL.label.equals(type)) {
                return BILATERAL;
            }
            else if (GAUSSIAN.label.equals(type)) {
                return GAUSSIAN;
            }
            else if (MEDIAN.label.equals(type)) {
                return MEDIAN;
            }
            else {
                return BOX;
            }
        }

        @Override
        public String toString() {
            return this.label;
        }
    }

    /**
     * Softens an image using one of several filters.
     * @param input The image on which to perform the blur.
     * @param type The blurType to perform.
     * @param doubleRadius The radius for the blur.
     * @param output The image in which to store the output.
     */
    private void blur(Mat input, SamplingPipeline.BlurType type, double doubleRadius,
                      Mat output) {
        int radius = (int)(doubleRadius + 0.5);
        int kernelSize;
        switch(type){
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                break;
            case MEDIAN:
                kernelSize = 2 * radius + 1;
                Imgproc.medianBlur(input, output, kernelSize);
                break;
            case BILATERAL:
                Imgproc.bilateralFilter(input, output, -1, radius, radius);
                break;
        }
    }

    /**
     * Segment an image based on hue, saturation, and value ranges.
     *
     * @param input The image on which to perform the HSL threshold.
     * @param hue The min and max hue
     * @param sat The min and max saturation
     * @param val The min and max value
     * @param out The image in which to store the output.
     */
    private void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }

    /**
     * Expands area of lower value in an image.
     * @param src the Image to erode.
     * @param kernel the kernel for erosion.
     * @param anchor the center of the kernel.
     * @param iterations the number of times to perform the erosion.
     * @param borderType pixel extrapolation method.
     * @param borderValue value to be used for a constant border.
     * @param dst Output Image.
     */
    private void cvErode(Mat src, Mat kernel, Point anchor, double iterations,
                         int borderType, Scalar borderValue, Mat dst) {
        if (kernel == null) {
            kernel = new Mat();
        }
        if (anchor == null) {
            anchor = new Point(-1,-1);
        }
        if (borderValue == null) {
            borderValue = new Scalar(-1);
        }
        Imgproc.erode(src, dst, kernel, anchor, (int)iterations, borderType, borderValue);
    }

    /**
     * Filter out an area of an image using a binary mask.
     * @param input The image on which the mask filters.
     * @param mask The binary image that is used to filter.
     * @param output The image in which to store the output.
     */
    private void mask(Mat input, Mat mask, Mat output) {
        mask.convertTo(mask, CvType.CV_8UC1);
        Core.bitwise_xor(output, output, output);
        input.copyTo(output, mask);
    }

    /**
     * Detects groups of pixels in an image.
     * @param input The image on which to perform the find blobs.
     * @param minArea The minimum size of a blob that will be found
     * @param circularity The minimum and maximum circularity of blobs that will be found
     * @param darkBlobs The boolean that determines if light or dark blobs are found.
     */
    private List<MatOfPoint> findBlobs(Mat input, double minArea, double[] circularity,
                           Boolean darkBlobs) {

//        FeatureDetector blobDet = FeatureDetector.create(FeatureDetector.SIMPLEBLOB);
//        blobDet.detect(input, blobList);

        ColorBlobDetector detector = new ColorBlobDetector();
        detector.process(input);
        return detector.getContours();
    }
}
