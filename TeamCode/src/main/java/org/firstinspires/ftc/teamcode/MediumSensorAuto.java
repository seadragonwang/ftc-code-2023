package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.OrcaRobot.LEFT;
import static org.firstinspires.ftc.teamcode.OrcaRobot.CENTER;
import static org.firstinspires.ftc.teamcode.OrcaRobot.RIGHT;
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
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(name="MediumSensorAuto")
public class MediumSensorAuto extends LinearOpMode {

    public final static double DIST_BETWEEN_SENSORS = 0.59;
    AprilTagDetection tagOfInterest = null;
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
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
    private void printDistanceTelemetry(SampleMecanumDrive drive){
        telemetry.addData("left distance in inches", drive.getLeftDistance());
        telemetry.addData("left front distance in inches", drive.getLeftFrontDistance());
        telemetry.addData("back distance in inches", drive.getBackDistance());
        telemetry.addData("right front distance in inches", drive.getRightFrontDistance());
        telemetry.addData("right back distance in inches", drive.getRightBackDistance());
        telemetry.update();
        RobotLog.i("left distance in inches" + drive.getLeftDistance());
        RobotLog.i("back distance in inches" + drive.getBackDistance());
        RobotLog.i("right front distance in inches" + drive.getRightFrontDistance());
        RobotLog.i("right back distance in inches" + drive.getRightBackDistance());
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
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = drive.pipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == CENTER || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    telemetry.update();
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                    telemetry.update();
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
                telemetry.update();
            }
        }
        printDistanceTelemetry(drive);
        waitForStart();

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
        double leftFrontDist = drive.getLeftFrontDistance();

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
                .addTemporalMarker(() -> {
                    drive.raiseSlider(0);
                })
                .waitSeconds(0.2)
                .build();
        drive.followTrajectorySequence(trajSeq14);
        TrajectorySequenceBuilder trajectorySequenceBuilder = drive.trajectorySequenceBuilder(trajSeq14.end());

        if(tagOfInterest.id ==LEFT){
            trajectorySequenceBuilder = trajectorySequenceBuilder.forward(12);
        }else if(tagOfInterest.id == CENTER){
            trajectorySequenceBuilder = trajectorySequenceBuilder.back(12);
        }else if(tagOfInterest.id == RIGHT){
            trajectorySequenceBuilder = trajectorySequenceBuilder.back(36);
        }else {
            trajectorySequenceBuilder = trajectorySequenceBuilder.forward(12);
        }
        drive.followTrajectorySequence(trajectorySequenceBuilder.build());
        if(isStopRequested()) return;


    }
}