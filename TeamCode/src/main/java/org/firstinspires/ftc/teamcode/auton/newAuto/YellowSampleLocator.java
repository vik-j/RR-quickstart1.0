package org.firstinspires.ftc.teamcode.auton.newAuto;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class YellowSampleLocator {

    public static Point getSamplePosition(Mat input) {
        Mat mask = new Mat();
        Mat edges = new Mat();

        // Convert to HSV and threshold for yellow
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv, new Scalar(20, 100, 100), new Scalar(30, 255, 255), mask);

        // Apply edge detection
        Imgproc.Canny(mask, edges, 50, 150);

        // Find contours
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(edges, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour (assuming it's the sample)
        double maxArea = 0;
        Point centroid = new Point(-1, -1);
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                Moments M = Imgproc.moments(contour);
                if (M.get_m00() != 0) {
                    centroid = new Point(M.get_m10() / M.get_m00(), M.get_m01() / M.get_m00());
                }
            }
        }

        return centroid; // Returns the sample’s position in pixels
    }
}
