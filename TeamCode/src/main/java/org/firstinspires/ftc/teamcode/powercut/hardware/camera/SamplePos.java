package org.firstinspires.ftc.teamcode.powercut.hardware.camera;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;

@Config
public class SamplePos extends OpenCvPipeline {

    public static int blurValue = 8; // Default blur value
    private Mat blurGaussianMat = new Mat();
    private Mat hsvMat = new Mat();

    // HSV thresholds
    public static Scalar lowerHSV = new Scalar(0.0, 82.0, 104.0);
    public static Scalar upperHSV = new Scalar(255.0, 255.0, 255.0);
    private Mat hsvBinaryMat = new Mat();

    private final ArrayList<MatOfPoint> contours = new ArrayList<>();
    private Mat hierarchy = new Mat();

    public static int minArea = 2000;
    public static int maxArea = 10000;

    private final ArrayList<MatOfPoint> contoursByArea = new ArrayList<>();
    private MatOfPoint2f contoursByArea2f = new MatOfPoint2f();
    private final ArrayList<RotatedRect> contoursByAreaRotRects = new ArrayList<>();

    // Line drawing parameters
    public static Scalar lineColor = new Scalar(255.0, 0.0, 0.0);
    public static int lineThickness = 3;

    private Mat inputRotRects = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        // Apply Gaussian blur
        Imgproc.GaussianBlur(input, blurGaussianMat, new Size((6 * blurValue) + 1, (6 * blurValue) + 1), blurValue);

        // Convert to HSV color space
        Imgproc.cvtColor(blurGaussianMat, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Threshold HSV values
        Core.inRange(hsvMat, lowerHSV, upperHSV, hsvBinaryMat);

        // Find contours
        contours.clear();
        Imgproc.findContours(hsvBinaryMat, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Filter contours by area
        contoursByArea.clear();
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area >= minArea && area <= maxArea) {
                contoursByArea.add(contour);
            }
        }

        // Convert contours to rotated rectangles
        contoursByAreaRotRects.clear();
        for (MatOfPoint points : contoursByArea) {
            contoursByArea2f.release();
            points.convertTo(contoursByArea2f, CvType.CV_32F);
            contoursByAreaRotRects.add(Imgproc.minAreaRect(contoursByArea2f));
        }

        // Draw rotated rectangles on the input frame
        input.copyTo(inputRotRects);
        for (RotatedRect rect : contoursByAreaRotRects) {
            if (rect != null) {
                Point[] rectPoints = new Point[4];
                rect.points(rectPoints);
                MatOfPoint matOfPoint = new MatOfPoint(rectPoints);

                Imgproc.polylines(inputRotRects, Collections.singletonList(matOfPoint), true, lineColor, lineThickness);
            }
        }

        return inputRotRects;
    }
}
