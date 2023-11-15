package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.Features2d;
import org.opencv.features2d.SimpleBlobDetector;
import org.opencv.features2d.SimpleBlobDetector_Params;
import org.opencv.imgproc.Imgproc;
import org.opencv.calib3d.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

import static org.opencv.core.Core.perspectiveTransform;

public class SpikeMarkDetector implements VisionProcessor {

    private int frame_width, frame_height;

    // which vertical percentage of the image to analyze
    private static final int VIEW_STARTING_Y_PERCENT = 25;
    private static final int VIEW_ENDING_Y_PERCENT = 70;

    // A copy of the telemetry object to sending messages
    // These messages can be seen in the EOVCSim program that
    // helps for testing the pipeline

    private Telemetry telemetry;

    // Stores the X,Y of the center of the image
    // and the minimum and maximum Y to be analyzed
    protected double centerX;
    protected double centerY;
    protected int minY, maxY;
    protected int imageWidth, imageHeight;

    // public variables to control how image is detected
    // These can be adjusted in EOCVSim
    //
    // Minimum bc square area Was 400, but blue square next to barrier can appear
    // smaller because partially hidden
    //
    // SU minimum area was 1000 and minimum sides 6, but when team cargo piece is placed on
    // square beside SU, it blocks some of SU tape so it appears as two pieces
    // so changed SU minimum area to 900 and minimum sides to 4, to allow a smaller part
    // of the SU tape to be allowed

    public int minThreshold = 137;   // Minimum level of red/blue to accept (0-255)
    public int maxThreshold = 255;   // Maximum level of red/blue to accept (0-255)
    public int minBCArea = 1000;     // Minimum area of pixels accepted for a barcode square
    public int  maxBCArea = 10000;    // Maximum area of pixels for a barcode square
    public int maxBCSides = 10;  // Maximum sides allow for a barcode square
    public int minSUArea = 900;    // Minimum area of pixel for piece of storage unit
    public int maxSUArea = 7000;   // Maximum area of pixels for a piece of storage unit
    public double epsilon = 3;
    public int minSUSides = 4;    // Minimum sides for a piece of storage unit
    public int maxSUSides = 12;     // Maximum sides for a piece of storage unti

    public boolean saveFrame = false;   // if set true, save frame and reset to false
    // These variables hold different version of the image from the camera
    // as they are processed

    private Mat blueThreshold;  // Image of just blue areas within the blue threshold
    private Mat redThreshold;   // Image of just red areas within the red threshold

    private Mat matYCrCb;       // The main image converted to YCrCb to get red and blue parts
    private Mat redChannel;     // The image with just the red part
    private Mat blueChannel;    // The image with just the blue part
    private Mat lumaChannel;    // just the luminance
    private Mat lumaThreshold;

    private Mat tmpMat;

    private List<MatOfPoint> redContours;   // A list of all the red shapes (contours) found
    private List<MatOfPoint> blueContours;  // A list of all the blue shapes (contours) found

    private SimpleBlobDetector m_detector;

    private boolean signalFound = false;
    private int signalX;
    private int signalY;

    public class VisionResult {
        int shapes = 0;
        String message = "";
    }

    public SpikeMarkDetector(Telemetry telemetry) {
        this.telemetry = telemetry;

        // Create all the Mat() object to hold the different images
        matYCrCb = new Mat();
        redChannel = new Mat();
        blueChannel = new Mat();
        lumaChannel = new Mat();

        blueThreshold = new Mat();
        redThreshold = new Mat();
        lumaThreshold = new Mat();

        tmpMat = new Mat();

        // Create the lists for holding the contours
        blueContours = new ArrayList<MatOfPoint>();
        redContours = new ArrayList<MatOfPoint>();

/*
        SimpleBlobDetector_Params params = new SimpleBlobDetector_Params();
        params.set_filterByArea(false);
        params.set_filterByColor(false);
        params.set_minArea( minSUArea );
        params.set_maxArea( maxSUArea );
        params.set_minThreshold(255 );
        params.set_maxThreshold(255);

        m_detector = SimpleBlobDetector.create( params );
*/

    }
                             // provides frame and camera info
                             @Override
    public void init(int width, int height, CameraCalibration calibration) {
         imageWidth = width;
         imageHeight = height;

         centerX = ((double) imageWidth / 2) - 0.5;
         centerY = ((double) imageHeight / 2) - 0.5;

         minY = imageHeight * VIEW_STARTING_Y_PERCENT / 100;
         maxY = imageHeight * VIEW_ENDING_Y_PERCENT / 100;
    }

    protected void finalize()
    {
        tmpMat.release();
        lumaThreshold.release();
        blueThreshold.release();
        redThreshold.release();
        lumaChannel.release();
        blueChannel.release();
        redChannel.release();
    }

    /**
     * Check one shape (contour) to see if it is a barcode square or a piece of the storage unit.
     *
     * @param contour
     * @return  True if the contour seems to be a barcode square
     */
    public boolean filterContours(Mat input, MatOfPoint contour) {
        double a = 0;
        boolean bRet = true;

        // Sometimes the shape isn't exactly a square and it might have too many sides so
        // the approxPolyDP function is used to simplify the shape to a polygon that is almost
        // the same. This makes the barcode shape usually be exactly 4 sided, or sometimes 5
        // sided but never really any more than that

        MatOfPoint2f poly2f = new MatOfPoint2f();
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
        MatOfPoint poly = new MatOfPoint();

        Imgproc.approxPolyDP(contour2f, poly2f, epsilon, true);

        // Now we know how many sides the shape has, and we work out how big it is
        // by getting the area in pixels
        int sideCount = poly2f.toArray().length;
        // get area of the poly

        poly.fromArray( poly2f.toArray() );
        int area = (int)Imgproc.contourArea(contour);

        // These lines can be uncommented when debugging
        // Log.i("PIPELINE", "contour area="+area+", sides=" + sideCount);
//        System.out.println("Shape: area="+area+", sides=" + sideCount);



        // If shape is right as edge of picture, it must be part of SU
        // and not a barcode, even if piece of SU that is visible is within the
        // shape parameters for a barcode square

        Rect r = Imgproc.boundingRect(contour);

        bRet = bRet && (area >= minBCArea && area <= maxBCArea);


        telemetry.addData("Shape" + (bRet ? " **" : " xx"), "area="+area+", sides=" + sideCount);

        //    if ( ( r.x < 2)   // shape is against left side
        //         || ( (r.x + r.width) > (imageWidth - 2) ) )  // or shape against right side
        //        return false;

        // Check if the shape has the right amount of sides for a barcode square
        if (bRet && sideCount > 3 && sideCount <= maxBCSides ) {

            // Uncomment this for debugging to draw the contour we are
            // examining onto the image
            List<MatOfPoint> polyPoints = new ArrayList<>();
            polyPoints.add(new MatOfPoint(poly2f.toArray() ) );

            Imgproc.drawContours(input, polyPoints, -1, new Scalar(255, 255, 0), 2);

            // FIND MOST TOP RIGHT THREE POINTS OF SHAPE
            // sort all points by distance from top right corner
            // select top three
            Point[] points = poly2f.toArray();

            List<Point> lp =
                    Arrays.stream(points).sorted(new Comparator<Point>() {
                        @Override
                        public int compare(Point point, Point t1) {
                            // distance from top right
                            return (int) Math.signum( Math.hypot( imageWidth - point.x, point.y)
                                    - Math.hypot( imageWidth - t1.x, t1.y) );

                        }
                    }).collect(Collectors.toList());

/*
            int pc = 3;
            for( Point pt : lp) {
                telemetry.addData("Point", pt.x + "," + pt.y);
                if (pc-- > 0) {
                    Imgproc.drawMarker( input, pt, new Scalar(255, 255, 0));
                }
            }
*/

            // get center point between 2nd and 3rd point
            Point p1 = lp.get(1);
            Point p2 = lp.get(2);
            Point ctr =
                    new Point(
                            Math.min(p1.x, p2.x ) +  Math.abs( p1.x - p2.x) / 2,
                            Math.min(p1.y, p2.y) + Math.abs( p1.y - p2.y) /2
                    );
            Imgproc.drawMarker( input, ctr, new Scalar(0, 255, 255));
            signalFound = true;
            signalX = (int)ctr.x;
            signalY = (int)ctr.y;

            telemetry.addData("Signal at", ctr.x + "," + ctr.y);

            // And if there are the right amount of sides, then
            // accept the item as a barcode square if the area
            // is also within range

            // Return true is the area is within range, or false if not
            bRet = true;

        }

        contour2f.release();
        poly.release();
        poly2f.release();

        // If we get this far, the contour was not a barcode square, so return false.
        return bRet;
    }

    // Process the frame mat
    // frame object can be updated
    // Return arbitrary Object containing processing results for use in
    // onDrawFrame later (userContext parameter)

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        telemetry.clear();
        // Convert image format to YCrCb format to make it easier
        // to extract red and blue channels

        Imgproc.cvtColor(frame, matYCrCb, Imgproc.COLOR_RGB2YCrCb);

        // Make a rectangle for the area we are not interested in, and blank it
            Rect r = new Rect();
            r.x = 0;
            r.y = 0;
            r.width = matYCrCb.width();
            r.height = minY;

            Imgproc.rectangle(matYCrCb, r, new Scalar(0, 0, 0), Imgproc.FILLED);

            r.y = maxY;
            r.height = matYCrCb.height() - maxY;
            Imgproc.rectangle(matYCrCb, r, new Scalar(0, 0, 0), Imgproc.FILLED);


        Core.extractChannel(matYCrCb, redChannel, 1);
        Core.extractChannel(matYCrCb, blueChannel, 2);
        Core.extractChannel(matYCrCb, lumaChannel, 0);

             Imgproc.threshold(redChannel, redThreshold, minThreshold, maxThreshold, Imgproc.THRESH_BINARY);

       //      redThreshold.copyTo(frame);


        blueContours.clear();
        redContours.clear();
        Imgproc.findContours(blueThreshold, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(redThreshold, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


        Imgproc.drawContours(frame, redContours, -1, new Scalar(0, 55, 255), 1);


        //    blueContours = blueContours.stream().filter(i -> filterContours(i)).collect(Collectors.toList());

        // same for red..
        redContours = redContours.stream().filter(i -> filterContours(frame, i)).collect(Collectors.toList());

     //   Imgproc.drawContours(frame, redContours, -1, new Scalar(255, 255, 0));

        //  redThreshold.copyTo(frame);
  //     if (1 == 1) { telemetry.update(); return null; }

        telemetry.update();

        VisionResult vr = new VisionResult();
        vr.shapes = 3;
        vr.message = "Hello";
        return vr;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();

        paint.setColor(Color.YELLOW);
        paint.setStrokeWidth(5);
        paint.setTextSize(20);

        canvas.drawLine(0, minY* scaleBmpPxToCanvasPx, onscreenWidth, minY* scaleBmpPxToCanvasPx, paint);
        canvas.drawLine(0, maxY * scaleBmpPxToCanvasPx -1, onscreenWidth, maxY * scaleBmpPxToCanvasPx -1, paint);
        canvas.drawText( ((VisionResult)userContext).message, 150, 50, paint);
    }
}
