package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;

public abstract class OrcaAutoBase extends OrcaRobot {
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.05;     // Larger is more responsive, but also less stable
    static final double     DRIVE_SPEED             = 0.6;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.5;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 2.0 ;    // How close must the heading get to the target before moving to next step.
    protected final File captureDirectory = AppUtil.ROBOT_DATA_DIR;
    protected SleeveDetectionPipeline pipeline   = null;
    protected OpenCvCamera    webcam        = null;
    protected BNO055IMU       imu           = null;      // Control/Expansion Hub IMU
    protected double          robotHeading  = 0;
    protected double          headingOffset = 0;
    protected double          headingError  = 0;
    protected double          targetHeading = 0;
    protected double  driveSpeed    = 0;
    protected double  turnSpeed     = 0;
    protected double  leftSpeed     = 0;
    protected double  rightSpeed    = 0;

    @Override
    protected void setup(){
        super.setup();

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        pipeline = new SleeveDetectionPipeline();
        webcam.setPipeline(pipeline);
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        motorFrontLeft.setPower(leftSpeed);
        motorBackLeft.setPower(leftSpeed);

        motorFrontRight.setPower(rightSpeed);
        motorBackRight.setPower(rightSpeed);
    }

    /**
     * Positive distanceInMilliMeter will move forward.
     * @param distanceInMilliMeter
     * @param speed
     */
    protected void driveDistance(int distanceInMilliMeter, double speed, double heading) {
        if(distanceInMilliMeter == 0) return;
        int distanceInCounts = (int) (distanceInMilliMeter * WHEEL_COUNTS_PER_MILLIMETER * 1.768);
        setDrivingMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int frCurrentPosition = motorFrontRight.getCurrentPosition();
        int flCurrentPosition = motorFrontLeft.getCurrentPosition();
        int brCurrentPosition = motorBackRight.getCurrentPosition();
        int blCurrentPosition = motorBackLeft.getCurrentPosition();
        int frTargetPosition = frCurrentPosition + distanceInCounts;
        int flTargetPosition = flCurrentPosition + distanceInCounts*-1;
        int brTargetPosition = brCurrentPosition + distanceInCounts;
        int blTargetPosition = blCurrentPosition + distanceInCounts*-1;
        motorFrontRight.setTargetPosition(frTargetPosition);
        motorFrontLeft.setTargetPosition(flTargetPosition);
        motorBackRight.setTargetPosition(brTargetPosition);
        motorBackLeft.setTargetPosition(blTargetPosition);
        setDrivingMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setPower(speed);
        motorFrontLeft.setPower(speed);
        motorBackRight.setPower(speed);
        motorBackLeft.setPower(speed);

        while (opModeIsActive() && isStillDriving()) {
//            sleep(50);
            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distanceInMilliMeter < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            moveRobot(speed, turnSpeed);
            sendTelemetry();
        }
    }

    public int roundAngle(double angle) {
        if (angle > 180) {
            return (int)(angle-360);
        } else if (angle < -180) {
            return (int)(angle+360);
        } else {
            return (int)angle;
        }
    }

    /**
     * Positive distanceInMilliMeter will slide to left.
     * @param distanceInMilliMeter
     * @param speed
     */
    protected void slide(int distanceInMilliMeter, double speed, double heading) {
        if(distanceInMilliMeter == 0) return;
        int distanceInCounts = (int) (distanceInMilliMeter * WHEEL_COUNTS_PER_MILLIMETER * 2.03);
        setDrivingMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int frCurrentPosition = motorFrontRight.getCurrentPosition();
        int flCurrentPosition = motorFrontLeft.getCurrentPosition();
        int brCurrentPosition = motorBackRight.getCurrentPosition();
        int blCurrentPosition = motorBackLeft.getCurrentPosition();
        int frTargetPosition = frCurrentPosition + distanceInCounts;
        int flTargetPosition = flCurrentPosition + distanceInCounts;
        int brTargetPosition = brCurrentPosition + distanceInCounts*-1;
        int blTargetPosition = blCurrentPosition + distanceInCounts*-1;
        motorFrontRight.setTargetPosition(frTargetPosition);
        motorFrontLeft.setTargetPosition(flTargetPosition);
        motorBackRight.setTargetPosition(brTargetPosition);
        motorBackLeft.setTargetPosition(blTargetPosition);
        setDrivingMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setPower(speed);
        motorFrontLeft.setPower(speed);
        motorBackRight.setPower(speed);
        motorBackLeft.setPower(speed);

        while (opModeIsActive() && isStillDriving()) {
            // Determine required steering to keep on heading
//            turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
//
//            // if driving in reverse, the motor correction also needs to be reversed
//            if (distanceInMilliMeter < 0)
//                turnSpeed *= -1.0;
//
//            // Apply the turning correction to the current driving speed.
//            moveRobot(speed, turnSpeed);
//            sendTelemetry();
            sleep(50);
        }
    }

    /**
     * positive degrees turns CCW, negative degrees turns CW
     */
    protected void turn(int degree, double speed) {
        if(degree == 0) return;
        int distanceInCounts = (int) (degree * CHASSIS_DIAMETER_IN_MILLIMETER*Math.PI*WHEEL_COUNTS_PER_MILLIMETER * 2.518/360);
        setDrivingMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int frCurrentPosition = motorFrontRight.getCurrentPosition();
        int flCurrentPosition = motorFrontLeft.getCurrentPosition();
        int brCurrentPosition = motorBackRight.getCurrentPosition();
        int blCurrentPosition = motorBackLeft.getCurrentPosition();
        int frTargetPosition = frCurrentPosition + distanceInCounts;
        int flTargetPosition = flCurrentPosition + distanceInCounts;
        int brTargetPosition = brCurrentPosition + distanceInCounts;
        int blTargetPosition = blCurrentPosition + distanceInCounts;
        motorFrontRight.setTargetPosition(frTargetPosition);
        motorFrontLeft.setTargetPosition(flTargetPosition);
        motorBackRight.setTargetPosition(brTargetPosition);
        motorBackLeft.setTargetPosition(blTargetPosition);
        setDrivingMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setPower(speed);
        motorFrontLeft.setPower(speed);
        motorBackRight.setPower(speed);
        motorBackLeft.setPower(speed);

        while (opModeIsActive() && isStillDriving()) {
//            // Determine required steering to keep on heading
//            turnSpeed = getSteeringCorrection(180, P_TURN_GAIN);
//
//            // Clip the speed to the maximum permitted value.
//            turnSpeed = Range.clip(turnSpeed, -speed, speed);
//
//            // Pivot in place by applying the turning correction
//            moveRobot(0, turnSpeed);
//            sendTelemetry();
            sleep(75);
        }
    }

    /**
     *  Display the various control parameters while driving
     *
     */
    protected void sendTelemetry() {
        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", getRawHeading(), robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.update();
    }
}
