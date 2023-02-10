package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name="MediumSensorAuto")
public class MediumSensorAuto extends LinearOpMode {
    SleevePosition position;

    public final static double DIST_BETWEEN_SENSORS = 0.59;

    public double [] calculateDistToHighJunction(double sensor1ToJunction, double sensor2ToJunction){
        double cosA = (sensor2ToJunction*sensor2ToJunction + DIST_BETWEEN_SENSORS*DIST_BETWEEN_SENSORS - sensor1ToJunction*sensor1ToJunction)/(2 * DIST_BETWEEN_SENSORS * sensor2ToJunction);
        double[] dist = new double[2];
        if(cosA < 0){
            dist[1] = (sensor2ToJunction*sensor2ToJunction - sensor1ToJunction*sensor1ToJunction - DIST_BETWEEN_SENSORS*DIST_BETWEEN_SENSORS)/(2*DIST_BETWEEN_SENSORS);
            dist[0] = Math.sqrt(sensor1ToJunction*sensor1ToJunction - dist[1]*dist[1]);
        }
        double cosB = (sensor1ToJunction*sensor1ToJunction + DIST_BETWEEN_SENSORS*DIST_BETWEEN_SENSORS - sensor2ToJunction*sensor2ToJunction)/(2 * DIST_BETWEEN_SENSORS * sensor1ToJunction);
        if(cosB < 0) {
            dist[1] = -DIST_BETWEEN_SENSORS-(sensor1ToJunction*sensor1ToJunction - sensor2ToJunction*sensor2ToJunction - DIST_BETWEEN_SENSORS*DIST_BETWEEN_SENSORS)/(2*DIST_BETWEEN_SENSORS);
            dist[0] = Math.sqrt(sensor2ToJunction*sensor2ToJunction - dist[1]*dist[1]);
        }
        dist[1] = -(sensor1ToJunction*sensor1ToJunction-sensor2ToJunction*sensor2ToJunction+DIST_BETWEEN_SENSORS*DIST_BETWEEN_SENSORS)/(2*DIST_BETWEEN_SENSORS);
        dist[0] = Math.sqrt(sensor1ToJunction*sensor1ToJunction - dist[1]*dist[1]);
        return dist;
    }

    public double [] calculateDistToLowJunction(double sensor1ToJunction, double sensor2ToJunction){
        double cosA = (sensor2ToJunction*sensor2ToJunction + DIST_BETWEEN_SENSORS*DIST_BETWEEN_SENSORS - sensor1ToJunction*sensor1ToJunction)/(2 * DIST_BETWEEN_SENSORS * sensor2ToJunction);
        double[] dist = new double[2];
        if(cosA < 0){
            dist[0] = (sensor2ToJunction*sensor2ToJunction - sensor1ToJunction*sensor1ToJunction - DIST_BETWEEN_SENSORS*DIST_BETWEEN_SENSORS)/(2*DIST_BETWEEN_SENSORS);
            dist[1] = Math.sqrt(sensor1ToJunction*sensor1ToJunction - dist[0]*dist[0]);
        }
        double cosB = (sensor1ToJunction*sensor1ToJunction + DIST_BETWEEN_SENSORS*DIST_BETWEEN_SENSORS - sensor2ToJunction*sensor2ToJunction)/(2 * DIST_BETWEEN_SENSORS * sensor1ToJunction);
        if(cosB < 0) {
            dist[0] = -DIST_BETWEEN_SENSORS-(sensor1ToJunction*sensor1ToJunction - sensor2ToJunction*sensor2ToJunction - DIST_BETWEEN_SENSORS*DIST_BETWEEN_SENSORS)/(2*DIST_BETWEEN_SENSORS);
            dist[1] = Math.sqrt(sensor2ToJunction*sensor2ToJunction - dist[0]*dist[0]);
        }
        dist[0] = -(sensor1ToJunction*sensor1ToJunction-sensor2ToJunction*sensor2ToJunction+DIST_BETWEEN_SENSORS*DIST_BETWEEN_SENSORS)/(2*DIST_BETWEEN_SENSORS);
        dist[1] = Math.sqrt(sensor1ToJunction*sensor1ToJunction - dist[0]*dist[0]);
        return dist;
    }

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

        double xStackCones = -10.75;
        double yStackCones = -48;
        double xMediumJunction = -10.5;
        double yMediumJunction = -16.50;
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPos)
//                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL/1.2))
//                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(MAX_ACCEL/1.2, MAX_ANG_VEL/1.2, 12.28))
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
                })
                .lineToConstantHeading(new Vector2d(-24, -28))
                .build();
        drive.followTrajectorySequence(trajSeq1);
        double rightFrontDist = drive.getRightFrontDistance();
        double rightBackDist = drive.getRightBackDistance();
        double backDist = drive.getBackDistance();
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .forward(40.78 - backDist)
                .strafeRight(31 - rightBackDist)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(-14, -28, Math.toRadians(-90)))
                .addTemporalMarker(() -> {
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_FIVE_CONES);
                })
                .lineToLinearHeading(new Pose2d(xStackCones, yStackCones, Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(trajSeq2);
        double leftDist = drive.getLeftDistance();
        backDist = drive.getBackDistance();

        yStackCones = yStackCones + 10 - leftDist;
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(trajSeq2.end())
                .strafeRight(10 - leftDist)
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
                })
                .lineToLinearHeading(new Pose2d(xMediumJunction, yMediumJunction, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(trajSeq3);

        backDist = drive.getBackDistance();

        printDistanceTelemetry(drive);
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .forward(40.5 - backDist)
                .build();
        drive.followTrajectorySequence(trajSeq4);

        leftDist = drive.getLeftDistance();
        rightFrontDist = drive.getRightFrontDistance();
        if(rightFrontDist >10){
            rightFrontDist = 5;
        }
        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(trajSeq4.end())
                .strafeLeft(5.4-rightFrontDist)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(xStackCones, yStackCones, Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(trajSeq5);
        leftDist = drive.getLeftDistance();

        yStackCones = yStackCones + 10 - leftDist;
        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(trajSeq5.end())
                .addTemporalMarker(() -> {
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_FOUR_CONES);
                })
                .waitSeconds(0.4)
                .strafeRight(10 - leftDist)
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
                })
                .lineToLinearHeading(new Pose2d(xMediumJunction, yMediumJunction, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(trajSeq6);

        backDist = drive.getBackDistance();
        printDistanceTelemetry(drive);
        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(trajSeq6.end())
                .forward(40.5 - backDist)
                .build();
        drive.followTrajectorySequence(trajSeq7);

        rightFrontDist = drive.getRightFrontDistance();
        if(rightFrontDist >10){
            rightFrontDist = 5;
        }
        TrajectorySequence trajSeq8 = drive.trajectorySequenceBuilder(trajSeq7.end())
                .strafeLeft(5.4-rightFrontDist)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
                .waitSeconds(0.2)
                .lineToLinearHeading(new Pose2d(xStackCones, yStackCones, Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(trajSeq8);
        leftDist = drive.getLeftDistance();

        yStackCones = yStackCones + 10 - leftDist;
        TrajectorySequence trajSeq9 = drive.trajectorySequenceBuilder(trajSeq8.end())
                .addTemporalMarker(() -> {
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_THREE_CONES);
                })
                .strafeRight(10 - leftDist)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
                })
                .lineToLinearHeading(new Pose2d(xMediumJunction, yMediumJunction, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(trajSeq9);

        backDist = drive.getBackDistance();

        printDistanceTelemetry(drive);
        TrajectorySequence trajSeq10 = drive.trajectorySequenceBuilder(trajSeq9.end())
                .forward(40.5 - backDist)
                .build();
        drive.followTrajectorySequence(trajSeq10);

        rightFrontDist = drive.getRightFrontDistance();
        if(rightFrontDist >10){
            rightFrontDist = 5;
        }
        TrajectorySequence trajSeq11 = drive.trajectorySequenceBuilder(trajSeq10.end())
                .strafeLeft(5.4-rightFrontDist)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_TWO_CONES);
                })
                .lineToLinearHeading(new Pose2d(xStackCones, yStackCones, Math.toRadians(180)))
                .build();
        drive.followTrajectorySequence(trajSeq11);
        leftDist = drive.getLeftDistance();

        yStackCones = yStackCones + 10 - leftDist;
        TrajectorySequence trajSeq12 = drive.trajectorySequenceBuilder(trajSeq11.end())
                .strafeRight(10 - leftDist)
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
                })
                .lineToLinearHeading(new Pose2d(xMediumJunction, yMediumJunction, Math.toRadians(90)))
                .build();
        drive.followTrajectorySequence(trajSeq12);

        backDist = drive.getBackDistance();
        printDistanceTelemetry(drive);
        TrajectorySequence trajSeq13 = drive.trajectorySequenceBuilder(trajSeq12.end())
                .forward(40.5 - backDist)
                .build();
        drive.followTrajectorySequence(trajSeq4);

        rightFrontDist = drive.getRightFrontDistance();
        if(rightFrontDist >10){
            rightFrontDist = 5;
        }
        printDistanceTelemetry(drive);
        TrajectorySequence trajSeq14 = drive.trajectorySequenceBuilder(trajSeq13.end())
                .strafeLeft(5.4-rightFrontDist)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
                .waitSeconds(0.2)
                .build();
        drive.followTrajectorySequence(trajSeq14);

//                .addTemporalMarker(() -> {
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_ONE_CONES);
//                })
//                .lineToLinearHeading(new Pose2d(xStackCones, yStackCones, Math.toRadians(180)))
//                .build();
//        drive.followTrajectorySequence(trajSeq14);
//        leftDist = drive.getLeftDistance();
//        TrajectorySequence trajSeq15 = drive.trajectorySequenceBuilder(trajSeq14.end())
//                .strafeRight(10 - leftDist)
//                .addTemporalMarker(() -> {
//                    drive.closeClaw();
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
//                })
//                .lineToLinearHeading(new Pose2d(xMediumJunction, yMediumJunction, Math.toRadians(90)))
//                .build();
//        drive.followTrajectorySequence(trajSeq15);
//        backDist = drive.getBackDistance();
//        printDistanceTelemetry(drive);
//        TrajectorySequence trajSeq16 = drive.trajectorySequenceBuilder(trajSeq15.end())
//                .forward(40.5 - backDist)
//                .build();
//        drive.followTrajectorySequence(trajSeq16);
//        leftDist = drive.getLeftDistance();
//        rightFrontDist = drive.getRightFrontDistance();
//        TrajectorySequence trajSeq17 = drive.trajectorySequenceBuilder(trajSeq16.end())
//                .strafeLeft(5.4-rightFrontDist)
//                .addTemporalMarker(() -> {
//                    drive.openClaw();
//                })
//                .waitSeconds(0.2)
//                .build();
//        drive.followTrajectorySequence(trajSeq17);
        //                .addTemporalMarker(() -> {
//                    drive.openClaw();
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_FOUR_CONES);
//                })
//                .lineToLinearHeading(new Pose2d(xStackCones, yStackCones, Math.toRadians(180)))
//                .waitSeconds(0.3)
//                .addTemporalMarker(() -> {
//                    drive.closeClaw();
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
//                })
//                .lineToLinearHeading(new Pose2d(xMediumJunction, yMediumJunction, Math.toRadians(90)))
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    drive.openClaw();
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_THREE_CONES);
//                })
//                .lineToLinearHeading(new Pose2d(xStackCones, yStackCones, Math.toRadians(180)))
//                .waitSeconds(0.3)
//                .addTemporalMarker(() -> {
//                    drive.closeClaw();
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
//                })
//                .lineToLinearHeading(new Pose2d(xMediumJunction+1, yMediumJunction+1, Math.toRadians(90)))
//                .addTemporalMarker(() -> {
//                    drive.openClaw();
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_TWO_CONES);
//                })
//                .lineToLinearHeading(new Pose2d(xStackCones, yStackCones, Math.toRadians(180)))
//                .waitSeconds(0.4)
//                .addTemporalMarker(() -> {
//                    drive.closeClaw();
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
//                })
//                .lineToLinearHeading(new Pose2d(xMediumJunction+1, yMediumJunction+1, Math.toRadians(90)))
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    drive.openClaw();
//                })
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_ONE_CONES);
//                })
//                .lineToLinearHeading(new Pose2d(xStackCones, yStackCones, Math.toRadians(180)))
//                .waitSeconds(0.4)
//                .addTemporalMarker(() -> {
//                    drive.closeClaw();
//                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(() -> {
//                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
//                })
//                .lineToLinearHeading(new Pose2d(xMediumJunction+1, yMediumJunction+1, Math.toRadians(90)))
//                .waitSeconds(0.2)
//                .addTemporalMarker(() -> {
//                    drive.openClaw();
//                })
//                .addTemporalMarker(() -> {
//                    drive.raiseSlider(0);
//                })
//                .build();
//        drive.followTrajectorySequence(trajSeq3);
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

        if(isStopRequested()) return;
//        SleevePosition position = pipeline.getAnalysis();


    }
    private void printDistanceTelemetry(SampleMecanumDrive drive){
        telemetry.addData("left distance in inches", drive.getLeftDistance());
        telemetry.addData("back distance in inches", drive.getBackDistance());
        telemetry.addData("right front distance in inches", drive.getRightFrontDistance());
        telemetry.addData("right back distance in inches", drive.getRightBackDistance());
        telemetry.update();
        RobotLog.i("left distance in inches" + drive.getLeftDistance());
        RobotLog.i("back distance in inches" + drive.getBackDistance());
        RobotLog.i("right front distance in inches" + drive.getRightFrontDistance());
        RobotLog.i("right back distance in inches" + drive.getRightBackDistance());
    }
}