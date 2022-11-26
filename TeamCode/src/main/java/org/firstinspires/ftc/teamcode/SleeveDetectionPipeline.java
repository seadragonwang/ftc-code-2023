package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

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

public class SleeveDetectionPipeline extends OpenCvPipeline
{
    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(312,251);
    static final int REGION_WIDTH = 55;
    static final int REGION_HEIGHT = 119;

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
    Point region_pointA = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x,
            REGION_TOPLEFT_ANCHOR_POINT.y);
    Point region_pointB = new Point(
            REGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    /*
     * Working variables
     */
    Mat region_r, region_g, region_b;
    Mat YCrCb = new Mat();
    Mat R = new Mat();
    Mat G = new Mat();
    Mat B = new Mat();
    int avg, avgr, avgg, avgb;
    Boolean pictureTaken = Boolean.FALSE;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile SleevePosition position = SleevePosition.LEFT;
    private int captureCounter = 0;
    private File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    private void saveBitmap(Bitmap bitmap) {
        File file = new File(captureDirectory, String.format(Locale.getDefault(), "webcam-frame-dup-%d.jpg", captureCounter++));
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
    void inputToYCrCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//            Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame)
    {
        /*
         * We need to call this in order to make sure the 'Cb'
         * object is initialized, so that the submats we make
         * will still be linked to it on subsequent frames. (If
         * the object were to only be initialized in processFrame,
         * then the submats would become delinked because the backing
         * buffer would be re-allocated the first time a real frame
         * was crunched)
         */
//            inputToCb(firstFrame);


        final String TAG = "bitmap";
        Mat img = new Mat();
        String photoPath = "/storage/self/primary/FIRST/data/webcam-frame-9.jpg";
//            BitmapFactory.Options options = new BitmapFactory.Options();
//            options.inSampleSize = 8;
//            final Bitmap bmp = BitmapFactory.decodeFile(photoPath);
//            Utils.bitmapToMat(bmp, img);
//            /*
//             * Submats are a persistent reference to a region of the parent
//             * buffer. Any changes to the child affect the parent, and the
//             * reverse also holds true.
//             */
//
//            Bitmap bmp2 = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
//            Utils.matToBitmap(firstFrame,bmp2);
//            saveBitmap(bmp2);
//            inputToYCrCb(firstFrame);
//
//            Core.extractChannel(YCrCb, R, 0);
//            Core.extractChannel(YCrCb, G, 1);
//            Core.extractChannel(YCrCb, B, 2);
//            region_r = R.submat(new Rect(region_pointA, region_pointB));
//            region_g = G.submat(new Rect(region_pointA, region_pointB));
//            region_b = B.submat(new Rect(region_pointA, region_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {
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
        if(true) {
            avg = (int) Core.mean(input).val[0];
            if (avg > 0) {
                pictureTaken = Boolean.TRUE;
                inputToYCrCb(input);
                Bitmap bmp2 = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
                Utils.matToBitmap(input, bmp2);
                saveBitmap(bmp2);
//                    inputToYCrCb(input);

                Core.extractChannel(input, R, 0);
                Core.extractChannel(input, G, 1);
                Core.extractChannel(input, B, 2);
                region_r = R.submat(new Rect(region_pointA, region_pointB));
                region_g = G.submat(new Rect(region_pointA, region_pointB));
                region_b = B.submat(new Rect(region_pointA, region_pointB));
//            Bitmap bmp3 = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
//            Utils.matToBitmap(input,bmp3);
//            saveBitmap(bmp3);
                /*
                 * Compute the average pixel value of each submat region. We're
                 * taking the average of a single channel buffer, so the value
                 * we need is at index 0. We could have also taken the average
                 * pixel value of the 3-channel image, and referenced the value
                 * at index 2 here.
                 */
                avgr = (int) Core.mean(region_r).val[0];
                avgg = (int) Core.mean(region_g).val[0];
                avgb = (int) Core.mean(region_b).val[0];
                RobotLog.i(String.join(",", Integer.toString(avgr), Integer.toString(avgg), Integer.toString(avgb)));
                System.out.println(String.join(",", Integer.toString(avgr), Integer.toString(avgg), Integer.toString(avgb)));
                int delta = avgr-avgg;
                if(avgb > avgg && avgb > avgr){
                    position = SleevePosition.LEFT;
                }else if(avgr > avgg && avgr > avgb){
                    position = SleevePosition.RIGHT;
                }else if(avgg > avgr && avgg > avgb){
                    position = SleevePosition.CENTER;
                }
            }
        }
        /*
         * Find the max of the 3 averages
         */
//            int maxOneTwo = Math.max(avg1, avg2);
//            int max = Math.max(maxOneTwo, avg3);
//
//            /*
//             * Now that we found the max, we actually need to go and
//             * figure out which sample region that value was from
//             */
//            if(max == avg1) // Was it from region 1?
//            {
//                position = SkystonePosition.LEFT; // Record our analysis
//
//                /*
//                 * Draw a solid rectangle on top of the chosen region.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region1_pointA, // First point which defines the rectangle
//                        region1_pointB, // Second point which defines the rectangle
//                        GREEN, // The color the rectangle is drawn in
//                        -1); // Negative thickness means solid fill
//            }
//            else if(max == avg2) // Was it from region 2?
//            {
//                position = SkystonePosition.CENTER; // Record our analysis
//
//                /*
//                 * Draw a solid rectangle on top of the chosen region.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region2_pointA, // First point which defines the rectangle
//                        region2_pointB, // Second point which defines the rectangle
//                        GREEN, // The color the rectangle is drawn in
//                        -1); // Negative thickness means solid fill
//            }
//            else if(max == avg3) // Was it from region 3?
//            {
//                position = SkystonePosition.RIGHT; // Record our analysis
//
//                /*
//                 * Draw a solid rectangle on top of the chosen region.
//                 * Simply a visual aid. Serves no functional purpose.
//                 */
//                Imgproc.rectangle(
//                        input, // Buffer to draw on
//                        region3_pointA, // First point which defines the rectangle
//                        region3_pointB, // Second point which defines the rectangle
//                        GREEN, // The color the rectangle is drawn in
//                        -1); // Negative thickness means solid fill
//            }

        /*
         * Render the 'input' buffer to the viewport. But note this is not
         * simply rendering the raw camera feed, because we called functions
         * to add some annotations to this buffer earlier up.
         */
        return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public SleevePosition getAnalysis()
    {
        return position;
    }
}