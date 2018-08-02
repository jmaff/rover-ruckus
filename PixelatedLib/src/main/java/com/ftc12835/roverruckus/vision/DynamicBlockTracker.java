package com.ftc12835.roverruckus.vision;

import com.acmerobotics.library.vision.Overlay;
import com.acmerobotics.library.vision.Tracker;
import com.acmerobotics.library.vision.VisionCamera;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class DynamicBlockTracker extends Tracker {
    // filter weights
    public static double ECCENTRICITY_WEIGHT = 2;
    public static double SOLIDITY_WEIGHT = 1;
    public static double AREA_WEIGHT = 0;
    public static double DISTANCE_WEIGHT = 0.005;
    public static double AREA_DIFF_WEIGHT = 5;

    private class Block {
        public final MatOfPoint contour;
        public final RotatedRect ellipse;
        public final Point centroid;
        public final double eccentricity, solidity, area, distance;

        public Block(MatOfPoint contour) {
            this.contour = contour;

            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            this.ellipse = Imgproc.fitEllipse(contour2f);
            this.eccentricity = ellipse.size.width / ellipse.size.height;
            this.area = Imgproc.contourArea(contour);
            Rect rect = Imgproc.boundingRect(contour);
            this.solidity = area / rect.area();

            Moments moments = Imgproc.moments(contour);
            centroid = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());

            distance = Math.sqrt(Math.pow(centroid.x - TARGET_POINT.x, 2) + Math.pow(centroid.y - TARGET_POINT.y, 2));
        }

        /**
         * lower is better
         */
        public double score() {
            double eccentricityError = Math.pow(Math.log(eccentricity), 2);
            double solidityError = Math.pow(Math.log(solidity) - Math.log(Math.PI / 4), 2);
            double areaError = Math.pow(TARGET_AREA - area, 2);
            return ECCENTRICITY_WEIGHT * eccentricityError + SOLIDITY_WEIGHT * solidityError + AREA_WEIGHT * areaError + DISTANCE_WEIGHT * distance;
        }

        public double radius() {
            return (ellipse.size.width + ellipse.size.height) / 2;
        }
    }

    public static final Comparator<Block> BLOCK_COMPARATOR = (lhs, rhs) -> Double.compare(lhs.score(), rhs.score());

    public static int OPEN_KERNEL_SIZE = 7;
    public static int CLOSE_KERNEL_SIZE = 13;

    // yellow HSV range
    public static int YELLOW_LOWER_HUE = 60, YELLOW_LOWER_SAT = 80, YELLOW_LOWER_VALUE = 80;
    public static int YELLOW_UPPER_HUE = 37, YELLOW_UPPER_SAT = 255, YELLOW_UPPER_VALUE = 255;

    public static int TARGET_AREA = 5200; // px^2
    public static int MIN_AREA = 500;

    public static Point TARGET_POINT = new Point(350, 200);

    public static int RESIZE_WIDTH = 480;

    private Mat resized, hsv, hue, saturation, value, yellow;
    private Mat temp, morphOpen, morphClose, hierarchy, openKernel, closeKernel;
    private int openKernelSize, closeKernelSize;
    private List<Block> lastBlocks;
    private double resizedWidth, resizedHeight;

    private void smartHsvRange(Mat src, Scalar lowerHsv, Scalar upperHsv, Mat dest) {
        if (lowerHsv.val[0] > upperHsv.val[0]) {
            Core.inRange(src, lowerHsv, new Scalar(180, upperHsv.val[1], upperHsv.val[2]), dest);
            if (temp == null) {
                temp = new Mat();
            }
            Core.inRange(src, new Scalar(0, lowerHsv.val[1], lowerHsv.val[2]), upperHsv, temp);
            Core.bitwise_or(dest, temp, dest);
        } else {
            Core.inRange(src, lowerHsv, upperHsv, dest);
        }
    }

    private List<Block> findBlocks(Mat mask) {
        if (morphClose == null) {
            morphClose = new Mat();
            morphOpen = new Mat();
            hierarchy = new Mat();
        }

        Imgproc.morphologyEx(mask, morphOpen, Imgproc.MORPH_OPEN, openKernel);
        addIntermediate("MorphOpen", morphOpen);
        Imgproc.morphologyEx(morphOpen, morphClose, Imgproc.MORPH_CLOSE, closeKernel);
        addIntermediate("MorphClose", morphClose);

        List<MatOfPoint> blockContours = new ArrayList<>();
        Imgproc.findContours(morphClose, blockContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Block> blocks = new ArrayList<>();
        for (MatOfPoint contour : blockContours) {
            if (Imgproc.contourArea(contour) >= MIN_AREA && contour.rows() >= 5) {
                blocks.add(new Block(contour));
            }
        }

        return blocks;
    }

    @Override
    public void init(VisionCamera camera) {

    }

    @Override
    public void processFrame(Mat frame, double timestamp) {
        resizedWidth = RESIZE_WIDTH;
        resizedHeight = (int) (frame.rows() * (resizedWidth / frame.cols()));

        if (resized == null) {
            resized = new Mat();
            hsv = new Mat();
            hue = new Mat();
            saturation = new Mat();
            value = new Mat();
            yellow = new Mat();
        }

        if (openKernel == null || openKernelSize != OPEN_KERNEL_SIZE) {
            if (openKernel != null) {
                openKernel.release();
            }
            openKernel = Mat.ones(OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE, CvType.CV_8U);
            openKernelSize = OPEN_KERNEL_SIZE;
        }

        if (closeKernel == null || closeKernelSize != CLOSE_KERNEL_SIZE) {
            if (closeKernel != null) {
                closeKernel.release();
            }
            closeKernel = Mat.ones(CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE, CvType.CV_8U);
            closeKernelSize = CLOSE_KERNEL_SIZE;
        }

        Imgproc.resize(frame, resized, new Size(resizedWidth, resizedHeight));

        Imgproc.GaussianBlur(resized, resized, new Size(5, 5), 0);

        addIntermediate("blurred", resized);

        Imgproc.cvtColor(resized, hsv, Imgproc.COLOR_BGR2HSV);

        Core.extractChannel(hsv, hue, 0);
        Core.extractChannel(hsv, saturation, 1);
        Core.extractChannel(hsv, value, 2);

        addIntermediate("hue", hue);
        addIntermediate("saturation", saturation);
        addIntermediate("value", value);

        Scalar yellowLowerHsv = new Scalar(YELLOW_LOWER_HUE, YELLOW_LOWER_SAT, YELLOW_LOWER_VALUE);
        Scalar yellowUpperHsv = new Scalar(YELLOW_UPPER_HUE, YELLOW_UPPER_SAT, YELLOW_UPPER_VALUE);
        smartHsvRange(hsv, yellowLowerHsv, yellowUpperHsv, yellow);

        addIntermediate("yellow", yellow);

        lastBlocks = findBlocks(yellow);

        Collections.sort(lastBlocks, BLOCK_COMPARATOR);
    }

    @Override
    public void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {
        overlay.setScalingFactor(imageWidth / resizedWidth);

        if (lastBlocks != null) {
            for (int i = 0; i < lastBlocks.size(); i++) {
                Block blockDetection = lastBlocks.get(i);
                overlay.strokeContour(blockDetection.contour, new Scalar(255, 255, 0), 4);
                overlay.putText(String.valueOf(i), Overlay.TextAlign.CENTER,
                        new Point(blockDetection.centroid.x, blockDetection.centroid.y), new Scalar(255, 255, 0), 30);
            }
        }

        overlay.fillCircle(TARGET_POINT, 10, new Scalar(0, 255, 255));

        overlay.setScalingFactor(1);
    }

    public int getBlocksDetected() {
        return lastBlocks != null ? lastBlocks.size() : 0;
    }
}
