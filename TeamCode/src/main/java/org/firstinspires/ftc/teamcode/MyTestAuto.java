package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name="MyTestAuto")
public class MyTestAuto extends LinearOpMode {
    SleevePosition position;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.openClaw();
        waitForStart();
        position = drive.pipeline.getAnalysis();
        telemetry.addData("pos", position);
        telemetry.update();
        Pose2d startPos = new Pose2d(-62, -28, 0);
        drive.setPoseEstimate(startPos);

        TrajectorySequence trajSeq;
        TrajectorySequenceBuilder trajSeqBuilder = drive.trajectorySequenceBuilder(startPos)
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
                })
                .forward(38)
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
                .waitSeconds(0.3)
                .forward(11.25)
                .addTemporalMarker(()->{// First stack cone
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_FIVE_CONES - 80);
                })
                .turn(Math.toRadians(182))
                .strafeLeft(19.5)
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(()->{
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_LOW_JUNCTION - 75);
                })
                .waitSeconds(0.2)
                .turn(-Math.toRadians(119))
//                .strafeLeft(leftSensor.getDistance(DistanceUnit.INCH)-3.5)
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
//                .waitSeconds(0.1)
                .addTemporalMarker(()->{// Second stack cone
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_FOUR_CONES - 80);
                })
                .turn(Math.toRadians(119))
                .addTemporalMarker(() -> {
                    drive.closeClaw();

                })
                .waitSeconds(0.3)
                .addTemporalMarker(()->{
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_LOW_JUNCTION-75);
                })
                .waitSeconds(0.2)
                .turn(-Math.toRadians(120))
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
//                .waitSeconds(0.2)
                .addTemporalMarker(()->{ // Third stack cone
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_THREE_CONES-80);
                })
                .waitSeconds(0.3)
                .turn(Math.toRadians(121))
                .strafeLeft(0.2)
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(()->{
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_LOW_JUNCTION-75);
                })
                .waitSeconds(0.2)
                .turn(-Math.toRadians(121))
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
//                .waitSeconds(0.2)
                .addTemporalMarker(()->{ // Fourth stack cone
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_TWO_CONES-80);
                })
                .waitSeconds(0.3)
                .turn(Math.toRadians(123))
                .strafeLeft(0.1)
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(()->{
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_LOW_JUNCTION-75);
                })
                .waitSeconds(0.2)
                .turn(-Math.toRadians(123))
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
//                .waitSeconds(0.2)
                .addTemporalMarker(()->{ // Fourth stack cone
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_ONE_CONES-80);
                })
                .turn(Math.toRadians(123))
                .strafeLeft(0.1)
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(()->{
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_LOW_JUNCTION-75);
                })
                .waitSeconds(0.2)
                .turn(-Math.toRadians(123))
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()->{
                    drive.raiseSlider(0);
                })
                .turn(Math.toRadians(29))
                .strafeRight(2);
        if(position == SleevePosition.LEFT){
//            trajSeq = trajSeqBuilder.splineTo(new Vector2d(-6,-6), Math.toRadians(90))
              trajSeq = trajSeqBuilder.forward(42.75)
                      .strafeLeft(3)
                .build();
        }else if(position == SleevePosition.RIGHT){
            trajSeq = trajSeqBuilder.back(5)
                    .build();
        }else{
            trajSeq = trajSeqBuilder.forward(19)
                    .build();
                    }

//                .addTemporalMarker(()->{ // Fourth stack cone
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_ONE_CONES);
//                })
//                .waitSeconds(0.3)
//                .turn(Math.toRadians(121))
//                .addTemporalMarker(() -> {
//                    drive.closeClaw();
//                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_LOW_JUNCTION);
//                })
//                .waitSeconds(0.3)
//                .turn(-Math.toRadians(121))
//                .addTemporalMarker(() -> {
//                    drive.openClaw();
//                })


        if(isStopRequested()) return;
//        SleevePosition position = pipeline.getAnalysis();
        drive.followTrajectorySequence(trajSeq);
    }
}