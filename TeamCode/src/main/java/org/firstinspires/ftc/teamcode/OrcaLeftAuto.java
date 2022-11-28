package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;

@Autonomous(name="OrcaLeftAuto")
public class OrcaLeftAuto extends OrcaAutoBase {
    private final File captureDirectory = AppUtil.ROBOT_DATA_DIR;

//    protected GyroSensor gyro;


    protected void raiseSlider3(int targetPos){
        raise.setTargetPosition(targetPos);
        raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        raise.setPower(1.0);
        while (raise.isBusy()) {
            sleep(100);
        }
    }

    protected void raiseSlider2(int targetPos){
        raise.setTargetPosition(targetPos);
        raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        raise.setPower(1.0);
        while (raise.isBusy()) {
            driveDistance(-300, 0.5, 0);

            driveDistance(1640, 0.1,0);
            openClaw();

        }
    }
    protected void prepareToPark(int targetPos, SleevePosition position){

        driveDistance(950, 0.6,180);
        if (position == SleevePosition.LEFT) {
            slide(-750, 0.6, 90);
        } else if (position == SleevePosition.RIGHT) {
            slide(700, 0.6, -90);
        }
        openClaw();
        raise.setTargetPosition(targetPos);
        raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        raise.setPower(1.0);
        while (raise.isBusy()) {
//            driveDistance(-300, 0.5);


            sleep(100);

        }
    }

    protected void raiseSlider1(int targetPos, SleevePosition position){
        raise.setTargetPosition(targetPos);
        raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        raise.setPower(1.0);
        while (raise.isBusy()) {
            slide(-130, 0.6,0);
            driveDistance(50, 0.8, 0);
            turn(180, 0.5);
//            driveDistance(100, -0.5, 0);

            driveDistance(-1590, 0.6,-180);

            openClaw();
            prepareToPark(0, position);

        }
    }

    @Override
    public void runOpMode()
    {

        setup();
        closeClaw();
        raise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        if (opModeIsActive()) {
            raiseSlider1(ARM_COUNTS_FOR_HIGH_JUNCTION, pipeline.getAnalysis());
            openClaw();
//            slide(-50, 0.5);

        }

//        if (pipeline.getAnalysis() == SleeverDetection.SleeveDetectionPipeline.SkystonePosition.CENTER) {
//            driveDistance(100, 0.2);
//            slide(150, 0.01);
//            driveDistance(700, 0.2);
////            turn(-50, 0.1);
//        } else if (pipeline.getAnalysis() == SleeverDetection.SleeveDetectionPipeline.SkystonePosition.RIGHT) {
//            driveDistance(100, 0.2);
//            slide(940, 0.01);
//            turn(4, 0.1);
//            driveDistance(700, 0.2);
//        } else {
//            driveDistance(150, 0.2);
//            slide(-600, 0.01);
//            turn(-3, 0.1);
//            driveDistance(680, 0.2);
//        }

    }
}
