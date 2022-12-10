package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="OrcaRightAuto")
public class OrcaRightAuto extends OrcaAutoBase {


    protected void raiseSlider1(int targetPos){
        raise.setTargetPosition(targetPos);
        raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        raise.setPower(1.0);
        while (raise.isBusy()) {
            slide(135, DRIVE_SPEED, 0);
            turn(roundAngle(-getRawHeading()), 0.4);
            driveDistance(880, 0.8, 0);


        }
        openClaw();
        sleep(100);
    }

    protected void raiseSlider2(int targetPos){
        raise.setTargetPosition(targetPos);
        raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        raise.setPower(1.0);
        while (raise.isBusy()) {
            sleep(50);
//            closeClaw();
        }

    }

    protected void raiseSlider3(int targetPos){
        raise.setTargetPosition(targetPos);
        raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        raise.setPower(1.0);
        while (raise.isBusy()) {
            sleep(100);
        }
//        turn(roundAngle((int) (60-getRawHeading())), 0.6);
//        slide(30, 0.5, 90);
        openClaw();
    }


    protected void raiseSlider4(int targetPos){
        raise.setTargetPosition(targetPos);
        raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        raise.setPower(1.0);
        while (raise.isBusy()) {
            sleep(50);
        }
    }


    @Override
    public void runOpMode() {
        setup();
        openClaw();
        robotHeading = getRawHeading();
        raise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        closeClaw();
        sleep(500);
        SleevePosition position = pipeline.getAnalysis();
        if (opModeIsActive()) {
            driveDistance(100, 0.6, 0);
            raiseSlider1(ARM_COUNTS_FOR_MEDIUM_JUNCTION);

//            slide(-50, 0.5, 0);
//            driveDistance(-240, DRIVE_SPEED, 0);
            if (position != SleevePosition.CENTER) {
                driveDistance(-350, 0.5, -180);
                if (position == SleevePosition.LEFT) {
                    turn(roundAngle((int)(90-getRawHeading())), 0.5);
                    driveDistance(545, 0.5, 90);
                    turn(roundAngle((int)(-getRawHeading())), 0.5);
                } else {
                    turn(roundAngle((int)(-90-getRawHeading())), 0.5);
                    driveDistance(540, 0.5, -90);
                    turn(roundAngle((int)(-getRawHeading())), 0.5);
                    driveDistance(100, 0.5, 90);
                }
            } else {
                driveDistance(-200, 0.5, -180);
            }
            raiseSlider3(0);



//            closeClaw();
//            sleep(400);
//            raiseSlider3(ARM_COUNTS_FOR_LOW_JUNCTION);
//            turn(-125, TURN_SPEED);
//            driveDistance(320, DRIVE_SPEED, -90);
//            openClaw();
//            raiseSlider4(ARM_COUNTS_FOR_FOUR_CONES);
//            closeClaw();
//            sleep(300);
//            raiseSlider3(ARM_COUNTS_FOR_LOW_JUNCTION);
//            turn(-125, TURN_SPEED);
//            openClaw();
//            raiseSlider4(0);
//            turn(35, TURN_SPEED);
//            turn(roundAngle(90-getRawHeading()), 0.4);
//
//            if(position == SleevePosition.LEFT){
//                driveDistance(620, DRIVE_SPEED, 90);
//            }else if(position == SleevePosition.RIGHT){
//                driveDistance(-570, DRIVE_SPEED, 90);
//            }
//            raiseSlider3(0);
//            sleep(200);
//            raiseSlider3(0);
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
