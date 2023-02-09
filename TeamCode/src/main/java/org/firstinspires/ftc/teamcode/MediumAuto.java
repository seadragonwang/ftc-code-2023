package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(name="MediumAuto")
public class MediumAuto extends LinearOpMode {
    SleevePosition position;
    AprilTagDetection tagOfInterest = null;
    boolean tagFound = false;
    static final double FEET_PER_METER = 3.28084;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.openClaw();
        waitForStart();
        ArrayList<AprilTagDetection> currentDetections = drive.pipeline.getLatestDetections();
//        position = drive.pipeline.getLatestDetections();
        for (AprilTagDetection tag : currentDetections) {
            if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                tagOfInterest = tag;
                tagFound = true;
                break;
            }
        }
        if (tagFound) {
            telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
            tagToTelemetry(tagOfInterest);
        } else {
            telemetry.addLine("Don't see tag of interest :(");

            if (tagOfInterest == null) {
                telemetry.addLine("(The tag has never been seen)");
            } else {
                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }
            telemetry.addData("pos", position);
            telemetry.update();
            Pose2d startPos = new Pose2d(-62, -28, 0);
            drive.setPoseEstimate(startPos);

            TrajectorySequence trajSeq;
            TrajectorySequenceBuilder trajSeqBuilder = drive.trajectorySequenceBuilder(startPos)
                    .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL / 1.2))
                    .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(MAX_ACCEL / 1.2, MAX_ANG_VEL / 1.2, 12.28))
//                .setAccelConstraint(MAX_ACCEL/2)
                    .addTemporalMarker(() -> {
                        drive.closeClaw();
                    })
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
                    })
                    .forward(48)
                    .lineToLinearHeading(new Pose2d(-11, -17, Math.toRadians(90)))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        drive.openClaw();
                    })
                    .waitSeconds(0.1)
                    .addTemporalMarker(() -> {
                        drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_FIVE_CONES);
                    })
                    .lineToLinearHeading(new Pose2d(-10.75, -48, Math.toRadians(180)))
                    .waitSeconds(0.25)
                    .addTemporalMarker(() -> {
                        drive.closeClaw();
                    })
                    .waitSeconds(0.25)
                    .addTemporalMarker(() -> {
                        drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
                    })
                    .waitSeconds(0.2)
                    .lineToLinearHeading(new Pose2d(-10.35, -15.75, Math.toRadians(90)))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        drive.openClaw();
                    })
                    .waitSeconds(0.1)
                    .addTemporalMarker(() -> {
                        drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_FOUR_CONES);
                    })
                    .lineToLinearHeading(new Pose2d(-11.25, -46.5, Math.toRadians(180)))
                    .waitSeconds(0.25)
                    .addTemporalMarker(() -> {
                        drive.closeClaw();
                    })
                    .waitSeconds(0.25)
                    .addTemporalMarker(() -> {
                        drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
                    })
                    .waitSeconds(0.2)
                    .lineToLinearHeading(new Pose2d(-10.5, -14.4, Math.toRadians(90)))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        drive.openClaw();
                    })
                    .waitSeconds(0.1)
                    .addTemporalMarker(() -> {
                        drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_THREE_CONES);
                    })
                    .lineToLinearHeading(new Pose2d(-12, -45.5, Math.toRadians(180)))
                    .waitSeconds(0.25)
                    .addTemporalMarker(() -> {
                        drive.closeClaw();
                    })
                    .waitSeconds(0.25)
                    .addTemporalMarker(() -> {
                        drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
                    })
                    .waitSeconds(0.2)
                    .lineToLinearHeading(new Pose2d(-10.7, -13.3, Math.toRadians(90)))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        drive.openClaw();
                    })
                    .waitSeconds(0.1)
                    .addTemporalMarker(() -> {
                        drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_TWO_CONES);
                    })
                    .lineToLinearHeading(new Pose2d(-12.6, -44.5, Math.toRadians(180)))
                    .waitSeconds(0.25)
                    .addTemporalMarker(() -> {
                        drive.closeClaw();
                    })
                    .waitSeconds(0.25)
                    .addTemporalMarker(() -> {
                        drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
                    })
                    .waitSeconds(0.2)
                    .lineToLinearHeading(new Pose2d(-10.95, -12.5, Math.toRadians(90)))
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        drive.openClaw();
                    })
                    .waitSeconds(0.1);
//                .addTemporalMarker(()->{// First stack cone
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_FIVE_CONES - 80);
//                })
////                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL/2))
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

            if (isStopRequested()) return;
//        SleevePosition position = pipeline.getAnalysis();
            drive.followTrajectorySequence(trajSeq);

        }
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}