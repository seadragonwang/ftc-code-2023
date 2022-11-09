package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;

public class SleevePipeline extends OpenCvPipeline {
    /*
     * An enum to define the skystone position
     */
    public enum ShippingElementPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar RED = new Scalar(255, 0, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(117, 329);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(483, 322);
    //        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(253, 98);
    static final int REGION_WIDTH = 135;
    static final int REGION_HEIGHT = 100;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//        Point region3_pointA = new Point(
//                REGION3_TOPLEFT_ANCHOR_POINT.x,
//                REGION3_TOPLEFT_ANCHOR_POINT.y);
//        Point region3_pointB = new Point(
//                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region1_Cb, region2_Cb, region3_Cb;
    Mat YCrCb = new Mat();
    Mat b = new Mat();
    int avg1, avg2, avg3;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile ShippingElementPosition position = ShippingElementPosition.LEFT;
    private int captureCounter = 0;
    private File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    private void saveBitmap(Bitmap bitmap) {
        File file = new File(captureDirectory, String.format(Locale.getDefault(), "webcam-frame-%d.jpg", captureCounter++));
        try {
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
//                telemetry.log().add("captured %s", file.getName());
            }
        } catch (IOException e) {
//            RobotLog.ee(TAG, e, "exception in saveBitmap()");
//            error("exception saving %s", file.getName());
        }

    }
    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input) {

//            File file = new File(AppUtil.ROBOT_DATA_DIR, String.format(Locale.getDefault(), "bitmap-frame-%d.jpg", 0));
//            try {
//                try (FileOutputStream outputStream = new FileOutputStream(file)) {
//                    bmp.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
////                    telemetry.log().add("captured %s", file.getName());
//                }
//            } catch (IOException e) {
//                RobotLog.ee(TAG, e, "exception in saveBitmap()");
//            }
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(input, b, 2);
    }

    @Override
    public void init(Mat firstFrame) {
//        final String TAG = "bitmap";
//        Mat img = new Mat();
//        String photoPath = "/storage/self/primary/FIRST/data/warehouse_2.jpg";
////            BitmapFactory.Options options = new BitmapFactory.Options();
////            options.inSampleSize = 8;
//        final Bitmap bmp = BitmapFactory.decodeFile(photoPath);
//        Utils.bitmapToMat(bmp, img);
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
        inputToCb(firstFrame);
        Bitmap bmp = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(firstFrame,bmp);
        saveBitmap(bmp);


        /*
         * Submats are a persistent reference to a region of the parent
         * buffer. Any changes to the child affect the parent, and the
         * reverse also holds true.
         */
        region1_Cb = b.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = b.submat(new Rect(region2_pointA, region2_pointB));

//            region3_Cb = b.submat(new Rect(region3_pointA, region3_pointB));
    }

    public Boolean isElement(int coordX, int coordY) { // provide top left corner
        for (int x = coordX; x < coordX + 122; x++) {
            for (int y = coordY; y < coordY + 87; y++) {
                final Point pointA = new Point(x, y);
                final Point pointB = new Point(x + 13, y + 13);
                Mat rectangle = b.submat(new Rect(pointA, pointB));
                int avgBlue = (int) Core.mean(rectangle).val[0];

                if (avgBlue > 190) {
                    return false; // this means cannot be shipping element
                }
            }
        }
        return true;
    }

    @Override
    public Mat processFrame(Mat input) {
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        System.out.println("started");
        Boolean SlotMID = isElement(144, 375); // for center
        Boolean Slot1 = isElement(424, 365); // for center
//        Boolean SlotMID = isElement(82,380); // for home
//        Boolean Slot1 = isElement(347,380); // for home
        /*
         * Overview of what we're doing:
         *
         * We first convert to YCrCb color space, from RGB color space.
         * Why do we do this? Well, in the RGB color space, chroma and
         * luma are intertwined. In YCrCb, chroma and luma are separated.
         * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
         * are Y, the luma channel (which essentially just a B&W image), the
         * Cr channel, which records the difference from red, and the Cb channel,
         * which records the difference from blue. Because chroma and luma are
         * not related in YCrCb, vision code written to look for certain values
         * in the Cr/Cb channels will not be severely affected by differing
         * light intensity, since that difference would most likely just be
         * reflected in the Y channel.
         *
         * After we've converted to YCrCb, we extract just the 2nd channel, the
         * Cb channel. We do this because stones are bright yellow and contrast
         * STRONGLY on the Cb channel against everything else, including SkyStones
         * (because SkyStones have a black label).
         *
         * We then take the average pixel value of 3 different regions on that Cb
         * channel, one positioned over each stone. The brightest of the 3 regions
         * is where we assume the SkyStone to be, since the normal stones show up
         * extremely darkly.
         *
         * We also draw rectangles on the screen showing where the sample regions
         * are, as well as drawing a solid rectangle over top the sample region
         * we believe is on top of the SkyStone.
         *
         * In order for this whole process to work correctly, each sample region
         * should be positioned in the center of each of the first 3 stones, and
         * be small enough such that only the stone is sampled, and not any of the
         * surroundings.
         */

        /*
         * Get the Cb channel of the input frame after conversion to YCrCb
         */
        inputToCb(input);
        /*
         * Compute the average pixel value of each submat region. We're
         * taking the average of a single channel buffer, so the value
         * we need is at index 0. We could have also taken the average
         * pixel value of the 3-channel image, and referenced the value
         * at index 2 here.
         */
        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];
//            avg3 = (int) Core.mean(region3_Cb).val[0];
        /*
         * Draw a rectangle showing sample region 1 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region1_pointA, // First point which defines the rectangle
                region1_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 2 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
                input, // Buffer to draw on
                region2_pointA, // First point which defines the rectangle
                region2_pointB, // Second point which defines the rectangle
                BLUE, // The color the rectangle is drawn in
                2); // Thickness of the rectangle lines

        /*
         * Draw a rectangle showing sample region 3 on the screen.
         * Simply a visual aid. Serves no functional purpose.
         */
//            Imgproc.rectangle(
//                    input, // Buffer to draw on
//                    region3_pointA, // First point which defines the rectangle
//                    region3_pointB, // Second point which defines the rectangle
//                    BLUE, // The color the rectangle is drawn in
//                    2); // Thickness of the rectangle lines


        /*
         * Find the max of the 3 averages
         */

        int max = Math.max(avg1, avg2);
//            int max = Math.max(maxOneTwo, avg3);

        /*
         * Now that we found the max, we actually need to go and
         * figure out which sample region that value was from
         */
//            if (200 > avg1) // Was it from region 1?
//            {
//                position = ShippingElementPosition.CENTER; // Record our analysis
//
//                /*
//                 * Draw a solid rectangle on top of the chosen region.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region1_pointA, // First point which defines the rectangle
//                        region1_pointB, // Second point which defines the rectangle
//                        RED, // The color the rectangle is drawn in
//                        -1); // Negative thickness means solid fill
//            } else if (200 > avg2) // Was it from region 2?
//            {
//                position = ShippingElementPosition.RIGHT; // Record our analysis
//
//                /*
//                 * Draw a solid rectangle on top of the chosen region.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region2_pointA, // First point which defines the rectangle
//                        region2_pointB, // Second point which defines the rectangle
//                        RED, // The color the rectangle is drawn in
//                        -1); // Negative thickness means solid fill
//            } else // Was it from region 3?
//            {
//                position = ShippingElementPosition.LEFT; // Record our analysis
//
//                /*
//                 * Draw a solid rectangle on top of the chosen region.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region1_pointA, // First point which defines the rectangle
//                        region2_pointB, // Second point which defines the rectangle
//                        RED, // The color the rectangle is drawn in
//                        -1); // Negative thickness means solid fill
//            }
        if (SlotMID) {
            position = ShippingElementPosition.CENTER;
        } else if (Slot1) {
            position = ShippingElementPosition.RIGHT;
        } else {
            position = ShippingElementPosition.LEFT;
        }
        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        System.out.println(time.time());
        return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public ShippingElementPosition getAnalysis() {
        return position;
    }
}