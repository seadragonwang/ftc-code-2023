package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;

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
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_HIGH_JUNCTION);
                })
                .forward(48)
                .lineToLinearHeading(new Pose2d(-12, -16.5, Math.toRadians(-90)))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_FIVE_CONES);
                })
                .lineToLinearHeading(new Pose2d(-10.75, -47.25, Math.toRadians(-180)))
                .waitSeconds(0.25)
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.1);
//                .addTemporalMarker(()->{// First stack cone
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_FIVE_CONES - 80);
//                })
//                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL/2))
////                .splineTo(new Vector2d(-9.5, -49), Math.toRadians(179.9))
//
////                .splineTo(new Vector2d(-8.39, -31.01), Math.toRadians(-165.53))
////                .splineTo(new Vector2d(-13.69, -46.38), Math.toRadians(-100.37))
////                .splineTo(new Vector2d(-12.81, -60.69), Math.toRadians(179.90))
//                .addTemporalMarker(() -> {
//                    drive.closeClaw();
//                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_HIGH_JUNCTION);
//                })
//                .lineToLinearHeading(new Pose2d(-2, -31, Math.toRadians(0)))
//                .addTemporalMarker(() -> {
//                    drive.openClaw();
//                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(()->{// Second stack cone
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_FOUR_CONES - 80);
//                });
//        if(position == SleevePosition.LEFT){
////            trajSeq = trajSeqBuilder.splineTo(new Vector2d(-6,-6), Math.toRadians(90))
//              trajSeq = trajSeqBuilder.forward(42.75)
//                      .strafeLeft(3)
//                .build();
//        }else if(position == SleevePosition.RIGHT){
//            trajSeq = trajSeqBuilder.back(5)
//                    .build();
//        }else{
//            trajSeq = trajSeqBuilder.forward(19)
//                    .build();
//                    }
        trajSeq = trajSeqBuilder.build();

        if(isStopRequested()) return;
//        SleevePosition position = pipeline.getAnalysis();
        drive.followTrajectorySequence(trajSeq);
    }
}