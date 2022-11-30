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
            sleep(50);
        }
    }

    protected void raiseSlider2(int targetPos){
        raise.setTargetPosition(targetPos);
        raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        raise.setPower(1.0);
        while (raise.isBusy()) {
            driveDistance(-320, DRIVE_SPEED, 0);
            turn(180, TURN_SPEED);
            slide(520, DRIVE_SPEED, 180);
        }
    }

    protected void raiseSlider3(int targetPos){
        raise.setTargetPosition(targetPos);
        raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        raise.setPower(1.0);
        while (raise.isBusy()) {
            sleep(100);
        }
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
        closeClaw();
        robotHeading = getRawHeading();
        raise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        SleevePosition position = pipeline.getAnalysis();
        if (opModeIsActive()) {
            raiseSlider1(ARM_COUNTS_FOR_HIGH_JUNCTION);
            slide(-130, DRIVE_SPEED, 0);

            driveDistance(1635, 0.7, 0);
            turn((int)-getRawHeading(), 0.4);
            openClaw();

            raiseSlider2(ARM_COUNTS_FOR_FIVE_CONES);
            closeClaw();
            sleep(400);
            raiseSlider3(ARM_COUNTS_FOR_LOW_JUNCTION);
            turn(-125, TURN_SPEED);
//            driveDistance(320, DRIVE_SPEED, -90);
            openClaw();
//            raiseSlider4(ARM_COUNTS_FOR_FOUR_CONES);
//            closeClaw();
//            sleep(300);
//            raiseSlider3(ARM_COUNTS_FOR_LOW_JUNCTION);
//            turn(-125, TURN_SPEED);
//            openClaw();
            raiseSlider4(0);
//            turn(35, TURN_SPEED);
            turn(90-(int)(getRawHeading()), 0.4);
            if(position == SleevePosition.LEFT){
                driveDistance(1100, DRIVE_SPEED, 90);
            }else if(position == SleevePosition.CENTER){
                driveDistance(580, DRIVE_SPEED, 90);
            }else{
                driveDistance(-50, DRIVE_SPEED, 90);
            }
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
