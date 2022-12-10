package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="MyTestAuto")
public class MyTestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPos = new Pose2d(-62, -40, 0);
        drive.setPoseEstimate(startPos);
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPos)
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(()->{
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_MEDIUM_JUNCTION);
                })
                .strafeLeft(5)
                .forward(37)
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(()->{// First stack cone
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_FIVE_CONES);
                })
                .forward(12)
                .turn(Math.toRadians(180))
                .strafeLeft(16.6)
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(()->{
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_LOW_JUNCTION);
                })
                .waitSeconds(0.3)
                .turn(-Math.toRadians(120))
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()->{// Second stack cone
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_FOUR_CONES);
                })
                .turn(Math.toRadians(120))
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(()->{
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_LOW_JUNCTION);
                })
                .waitSeconds(0.3)
                .turn(-Math.toRadians(121))
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(()->{ // Third stack cone
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_THREE_CONES);
                })
                .waitSeconds(0.3)
                .turn(Math.toRadians(121))
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(()->{
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_LOW_JUNCTION);
                })
                .waitSeconds(0.3)
                .turn(-Math.toRadians(121))
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(()->{ // Fourth stack cone
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_TWO_CONES);
                })
                .waitSeconds(0.3)
                .turn(Math.toRadians(121))
                .addTemporalMarker(() -> {
                    drive.closeClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(()->{
                    drive.raiseSlider(OrcaRobot.ARM_COUNTS_FOR_LOW_JUNCTION);
                })
                .waitSeconds(0.3)
                .turn(-Math.toRadians(121))
                .addTemporalMarker(() -> {
                    drive.openClaw();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(()->{
                    drive.raiseSlider(0);
                })
                .turn(Math.toRadians(31))
                .strafeRight(2)
                .forward(42)
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
                .build();
        drive.openClaw();
        waitForStart();
        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajSeq);

    }
}
