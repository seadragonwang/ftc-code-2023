/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;

/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */
@Autonomous(name="auto.exe (crish wnag > yiksen li)")
public class SleeverDetection extends LinearOpMode
{
    OpenCvWebcam webcam;

    SleeveDetectionPipeline pipeline;
    public static final int CHASSIS_LENGTH_IN_MILLIMETER = 304;
    public static final int CHASSIC_WIDTH_IN_MILLIMETER = 352;
    public static final int FIELD_LENGTH_IN_MILLIMETER = 3590;
    public static final int SHIPPING_HUB_RADIUS_IN_MILLIMETER = 226;
    public static final int CAROUSAL_RADIUS_IN_MILLIMETER = 191;
    public static final int RABBIT_POSITION_MIN_IN_MILLIMETER = 700;
    public static final double TURN_DISTANCE_PER_DEGREE = 4.05878;
    public static final int RABBIT_POSITION_MAX_IN_MILLIMETER = 800;
    public static final int HALF_CHASSIS_WIDTH_IN_MILLIMETER = 114;
    public static final int HALF_CHASSIS_LENGTH_IN_MILLIMETER = 219;
    public static final int BLUE_SHIPPING_HUB_X_IN_MILLIMETER = 1187;
    public static final int BLUE_SHIPPING_HUB_Y_IN_MILLIMETER = 1543;
    public static final int RED_SHIPPING_HUB_X_IN_MILLIMETER = 2375;
    public static final int RED_SHIPPING_HUB_Y_IN_MILLIMETER = 1543;
    public static final int STORAGE_START_X_IN_MILLIMETER = 630;
    public static final int STORAGE_START_Y_IN_MILLIMETER = 0;
    public static final int STORAGE_END_X_IN_MILLIMETER = 1150;
    public static final int STORAGE_END_Y_IN_MILLIMETER = 630;
    public static final int STORAGE_CENTER_X_IN_MILLIMETER = 890;
    public static final int DISTANCE_FROM_CAROUSAL_SPINNER_TO_RIGHT_DISTANCE_SENSOR = 100;
    public static final int DISTANCE_FROM_CAROUSAL_SPINNER_TO_FRONT_DISTANCE_SENSOR = 100;
    public static final double ANGLE_RATIO = 1.4;
    public static final double SLIDE_RATIO = 1;
    public static final double CHASSIS_RADIUS = 203;
    public static final double COUNTS_PER_ENCODER_REV = 28;
    public static final double WHEEL_GEAR_RATIO = 19.203208556149733;
    //    public static final double ARM_GEAR_RATIO = 139.13824192336588;
    public static final double WHEEL_DIAMETER_MILLIMETTER = 96;
    public static final int WHEEL_MOTOR_SPPED_IN_RPM = 312;
    //    public static final int ARM_MOTOR_SPPED_IN_RPM = 43;
    public static final double WHEEL_COUNTS_PER_DEGREE = COUNTS_PER_ENCODER_REV * WHEEL_GEAR_RATIO / 360;
    public static final double WHEEL_FULL_SPEED_IN_COUNTS = COUNTS_PER_ENCODER_REV * WHEEL_GEAR_RATIO * WHEEL_MOTOR_SPPED_IN_RPM;
    //    public static final double ARM_COUNTS_PER_DEGREE = COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO / 360;
//    public static final double ARM_FULL_SPEED_IN_COUNTS = COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO * ARM_MOTOR_SPPED_IN_RPM;
    public static final double COUNTS_PER_MILLIMETER = (COUNTS_PER_ENCODER_REV * WHEEL_GEAR_RATIO) / (WHEEL_DIAMETER_MILLIMETTER * Math.PI);
    //    public DistanceSensor frontDistanceSensor;
//    public DistanceSensor leftDistanceSensor;
//    public DistanceSensor rightDistanceSensor;
    public DcMotorEx motorFrontLeft;
    public DcMotorEx motorFrontRight;
    public DcMotorEx motorBackLeft;
    public DcMotorEx motorBackRight;

    public Servo claw;
    public void setDrivingMotorMode(DcMotor.RunMode mode) {
        motorFrontRight.setMode(mode);
        motorFrontLeft.setMode(mode);
        motorBackRight.setMode(mode);
        motorBackLeft.setMode(mode);
    }
    public boolean isStillDriving() {
        return motorFrontLeft.isBusy() || motorFrontRight.isBusy() || motorBackLeft.isBusy() || motorBackRight.isBusy();
    }
    //    public void setDrivingMotorMode(DcMotor.RunMode mode) {
//        motorFrontRight.setMode(mode);
//        motorFrontLeft.setMode(mode);
//        motorBackRight.setMode(mode);
//        motorBackLeft.setMode(mode);
//    }
    protected void driveDistance(int distanceInMilliMeter, double speed) {
        if(distanceInMilliMeter == 0) return;
        int direction = distanceInMilliMeter / Math.abs(distanceInMilliMeter);
        int distanceInCounts = (int) (distanceInMilliMeter * COUNTS_PER_MILLIMETER);
        setDrivingMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setTargetPosition(distanceInCounts);
        motorFrontLeft.setTargetPosition(-distanceInCounts);
        motorBackRight.setTargetPosition(distanceInCounts);
        motorBackLeft.setTargetPosition(-distanceInCounts);
        setDrivingMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorFrontLeft.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackLeft.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);

        while (isStillDriving()) {
            sleep(100);
//            // telemetry.addData("Front right motor speed", motorFrontRight.getVelocity());
//            // telemetry.addData("Front left motor speed", motorFrontLeft.getVelocity());
//            // telemetry.addData("Back right speed", motorBackRight.getVelocity());
//            // telemetry.addData("Back left speed", motorBackLeft.getVelocity());
//            // telemetry.update()
        }
    }
    /*
    positive degrees turns CCW, negative degrees turns CW
     */
    protected void turn(int degrees, double speed) {
        if(degrees == 0) return;
        int direction = degrees / Math.abs(degrees);
        int distanceInCounts = (int) (degrees * TURN_DISTANCE_PER_DEGREE * 1.35 * COUNTS_PER_MILLIMETER);
        setDrivingMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setTargetPosition(distanceInCounts);
        motorFrontLeft.setTargetPosition(distanceInCounts);
        motorBackRight.setTargetPosition(distanceInCounts);
        motorBackLeft.setTargetPosition(distanceInCounts);
        setDrivingMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorFrontLeft.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackLeft.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);

        while (isStillDriving()) {
            sleep(100);
//            // telemetry.addData("Front right motor speed", motorFrontRight.getVelocity());
//            // telemetry.addData("Front left motor speed", motorFrontLeft.getVelocity());
//            // telemetry.addData("Back right speed", motorBackRight.getVelocity());
//            // telemetry.addData("Back left speed", motorBackLeft.getVelocity());
//            // telemetry.update()
        }
    }
    protected void slide(int distanceInMilliMeter, double speed) {
        if(distanceInMilliMeter == 0) return;
        int direction = distanceInMilliMeter / Math.abs(distanceInMilliMeter);
        int distanceInCounts = (int) (distanceInMilliMeter * COUNTS_PER_MILLIMETER);
        setDrivingMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setTargetPosition(-distanceInCounts);
        motorFrontLeft.setTargetPosition(-distanceInCounts);
        motorBackRight.setTargetPosition(distanceInCounts);
        motorBackLeft.setTargetPosition(distanceInCounts);
        setDrivingMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorFrontLeft.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackLeft.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);

        while (isStillDriving()) {
            sleep(100);
//            // telemetry.addData("Front right motor speed", motorFrontRight.getVelocity());
//            // telemetry.addData("Front left motor speed", motorFrontLeft.getVelocity());
//            // telemetry.addData("Back right speed", motorBackRight.getVelocity());
//            // telemetry.addData("Back left speed", motorBackLeft.getVelocity());
//            // telemetry.update()
        }
    }
    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SleeveDetectionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("backRight");
        Servo claw = hardwareMap.servo.get("claw");
        Servo claw2 = hardwareMap.servo.get("claw2");
        claw.setPosition(0.72);
        claw2.setPosition(0.32);
        waitForStart();
//        sleep(2000);
        if (pipeline.getAnalysis() == SleeveDetectionPipeline.SkystonePosition.CENTER) {
            driveDistance(100, 0.2);
            slide(150, 0.01);
            driveDistance(700, 0.2);
//            turn(-50, 0.1);
        } else if (pipeline.getAnalysis() == SleeveDetectionPipeline.SkystonePosition.RIGHT) {
            driveDistance(100, 0.2);
            slide(940, 0.01);
            turn(4, 0.1);
            driveDistance(700, 0.2);
        } else {
            driveDistance(150, 0.2);
            slide(-600, 0.01);
            turn(-3, 0.1);
            driveDistance(680, 0.2);
        }
//        while (opModeIsActive())
//        {
//            telemetry.addData("Analysis", pipeline.getAnalysis());
//            telemetry.update();
//
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(50);
//        }
    }
    public static class SleeveDetectionPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum SkystonePosition
        {
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION_TOPLEFT_ANCHOR_POINT = new Point(226,271);
        static final int REGION_WIDTH = 46;
        static final int REGION_HEIGHT = 112;

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
        private volatile SkystonePosition position = SkystonePosition.LEFT;
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
                        position = SkystonePosition.LEFT;
                    }else if(avgr > avgg && avgr > avgb){
                        position = SkystonePosition.RIGHT;
                    }else if(avgg > avgr && avgg > avgb){
                        position = SkystonePosition.CENTER;
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
        public SkystonePosition getAnalysis()
        {
            return position;
        }
    }
}