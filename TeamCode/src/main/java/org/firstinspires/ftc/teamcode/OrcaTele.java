package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="OrcaTele",group="")
public class OrcaTele extends OrcaRobot {
    static final double     DRIVE_SPEED             = 0.7;     // Max driving speed for better distance accuracy.
    static final double SLIDE_SPEED = 0.85;

    /**
     * Power is positive, robot will slide left, otherwise slide right
     * @param power
     */
    protected void slideByPower(double power){
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(-power);
    }

    /**
     * Power is positive, robot will move forward, otherwise move backward.
     * @param power
     */
    protected void driveByPower(double power){
//        double newPower;
//        if(abs(power) < 0.01) {
//            newPower = 0;
//        } else {
//            newPower = power;
//        }
        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(power);
    }

    protected void turnByPower(double power){
        motorFrontLeft.setPower(-power / 2);
        motorBackLeft.setPower(-power / 2);
        motorFrontRight.setPower(-power / 2);
        motorBackRight.setPower(-power / 2);
    }

    protected void operate(){
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y; // Counteract imperfect strafing
        if (gamepad2.x) {
            openClaw();
            sleep(300);
            raiseSlider(0);
        } else if (gamepad2.left_bumper) {
            closeClaw();
        } else if (gamepad2.right_trigger > 0.3) {
            openClaw();
        }
        if (abs(x) > abs(y)) {
            y = 0;
        } else {
            x = 0;
        }
        double rx = gamepad1.right_stick_x;

        if(gamepad1.x){
            slideByPower(0.9);
        }else if(gamepad1.b){
            slideByPower(-0.9);
        }else if(gamepad1.y){
            driveByPower(DRIVE_SPEED);
        }else if(gamepad1.a){
            driveByPower(-DRIVE_SPEED);
        }else{
            driveByPower(0);
        }
        if (x == 0 && y == 0) {
            if (rx != 0) {
                turnByPower(rx);
            }
        } else if (x != 0) {
            slideByPower(-x*SLIDE_SPEED);
        } else {
            driveByPower(-y*DRIVE_SPEED);
        }
    }

    protected void raiseSlider(int targetPos){
        raise.setTargetPosition(targetPos);
        raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        raise.setVelocity(ARM_FULL_SPEED_IN_COUNTS);
        while (raise.isBusy()) {
            operate();
            sleep(50);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        openClaw();
//        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        if (isStopRequested()) return;



        raise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (opModeIsActive()) {

            raise.setTargetPositionTolerance(100);
            operate();

            int currentRaisedPosition = raise.getCurrentPosition();
            telemetry.addData("armPos", currentRaisedPosition);
            int raiseStep = 0;
            if(gamepad2.dpad_down) {
                raiseStep = 100;
            } else if (gamepad2.dpad_up) {
                raiseStep = -100;
            } else {
                raiseStep = 0;
            }
            int targetRaise = currentRaisedPosition;
            if (gamepad2.y) { // assuming bottom is 0, negative is up
                targetRaise = ARM_COUNTS_FOR_LOW_JUNCTION - 100;
            } else if (gamepad2.a) {
                targetRaise = ARM_COUNTS_FOR_HIGH_JUNCTION - 100;
            } else if (gamepad2.b) {
                targetRaise = ARM_COUNTS_FOR_MEDIUM_JUNCTION - 100;
            } else if (gamepad2.right_bumper){
                targetRaise = 0;
            } else {
                targetRaise = currentRaisedPosition + raiseStep;
            }

            raiseSlider(targetRaise);
            telemetry.addData("clawPos", claw.getPosition());
            telemetry.addData("claw2Pos", claw2.getPosition());
            telemetry.update();
        }
    }
}
