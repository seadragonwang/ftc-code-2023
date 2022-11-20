//Don't touch!!!
//Tele2
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="OrcaTele2",group="")
public class StupiderTele extends LinearOpMode {
    public static final double COUNTS_PER_ENCODER_REV = 28;
    public static int ARM_VERTIAL_POSITION = 1109;
    public static int ARM_FRONT_HORIZONTAL = 2041;
    public static int ARM_BACK_HORIZONTAL = 177;
    // pulley circumference is 112 mm
    public static final double ARM_GEAR_RATIO = 19.2;
    public static final double WHEEL_DIAMETER_MILLIMETTER = 96;
    public static final int WHEEL_MOTOR_SPPED_IN_RPM = 312;
    public static final int ARM_MOTOR_SPPED_IN_RPM = 312;
    public static final double ARM_COUNTS_PER_DEGREE = COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO / 360;
    public static final double ARM_FULL_SPEED_IN_COUNTS = COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO * ARM_MOTOR_SPPED_IN_RPM;
    @Override
    public void runOpMode() throws InterruptedException {

//        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
//        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distanceSensor;
        boolean runSlowMo = true;
//        DcMotor slider = hardwareMap.dcMotor.get("slider");
//        DcMotorEx arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
//        Servo hand = hardwareMap.servo.get("hand");
//        hand.setPosition(0.999);
//        boolean handUp = false;
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");
        DcMotorEx raise = (DcMotorEx) hardwareMap.dcMotor.get("raise");

//        Servo claw = hardwareMap.servo.get("claw");
//        hand.setPosition(0.36);
//        hand.setPosition(0.75);
//        hand.setDirection(Servo.Direction.REVERSE);
//        DcMotor carousel = hardwareMap.dcMotor.get("carousel");


//        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm"); // Minimum -2, Maximum -1272
        Servo claw = hardwareMap.servo.get("claw");
        Servo claw2 = hardwareMap.servo.get("claw2");

//        Servo grabber = hardwareMap.servo.get("grabber");

//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setTargetPosition(1);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setVelocity(ARM_FULL_SPEED_IN_COUNTS);
//        claw.setPosition(0.8);
        waitForStart();
        boolean grabberIsOpen = false;
        double GRABBER_SPEED = 0.02;
        double grabber_pos = 1;
        if (isStopRequested()) return;
//        claw.setPosition(0);
        boolean isOpen = false;
        boolean clawPositive = true;
        double handPos = 0;
        raise.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (opModeIsActive()) {
//            boolean isOpen = false;
//            if (gamepad2.y) {
//                if (handPos == 0.36) {
//                    handPos = 0.75;
//                } else {
//                    handPos = 0.36;
//                }
//            }
//            if (gamepad2.a) {
//                handPos = 0.6;
//            }
//
//            hand.setPosition(handPos);
            if (gamepad2.x) {
                claw.setPosition(1);
                claw2.setPosition(0);
            } else if (gamepad2.left_bumper) {
                claw.setPosition(0.65);
                claw2.setPosition(0.35);
            }
//            if (gamepad2.dpad_left) {
//                handPos += 0.02;
//            }
//            if (gamepad2.dpad_right) {
//                handPos -= 0.02;
//            }
//            claw.setPosition(0.3);
//            if (gamepad2.right_bumper || gamepad2.y) {
//                grabberIsOpen = !grabberIsOpen;
//            }
//            if (grabberIsOpen) {
//                grabber.setPosition(0.1);
//            } else {
//                grabber.setPosition(-0.02);
//            }



//            int currentArmPosition = arm.getCurrentPosition();
//            int armStep = 0;
//            if(gamepad2.dpad_down){
//                armStep = -200;
//            }else if(gamepad2.dpad_up){
//                armStep = 200;
//            }else{
//                armStep = 0;
////                armStep = (int)-gamepad2.left_stick_x*50;
//            }
//            int currentTurnPosition = turn.getCurrentPosition();
//            int turnStep = 50;
//            int targetPosition = armStep + currentArmPosition;
//            arm.setTargetPosition(targetPosition);
//            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            arm.setVelocity(ARM_FULL_SPEED_IN_COUNTS);
//            int targetTurn = currentTurnPosition + (int)(50*((gamepad2.right_trigger-gamepad2.left_trigger)));
//            turn.setTargetPosition(targetTurn);
//            turn.setVelocity(80 * (gamepad2.right_trigger-gamepad2.left_trigger));
            raise.setTargetPositionTolerance(100);
            double y = gamepad1.left_stick_x;
            double x = gamepad1.left_stick_y * 1.1; // Counteract imperfect strafing
            if (Math.abs(x) > Math.abs(y)) {
                y = 0;
            } else {
                x = 0;
            }
            double rx = gamepad1.right_stick_x;
            if (true) {
                if (x != 0 || y != 0) {
                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = -(y - x+rx) / denominator; // -1
                    double backLeftPower = -(-y - x+rx) / denominator; // 1
                    double frontRightPower = (-y - x+rx) / denominator; // -1
                    double backRightPower = (y - x+rx) / denominator; // 1
                    motorFrontLeft.setPower(frontLeftPower/2);
                    motorBackLeft.setPower(backLeftPower/2);
                    motorFrontRight.setPower(frontRightPower/2);
                    motorBackRight.setPower(backRightPower/2);
                } else {
                    motorFrontLeft.setPower(-rx/2);
                    motorBackLeft.setPower(-rx/2);
                    motorFrontRight.setPower(-rx/2);
                    motorBackRight.setPower(-rx/2);
                }
            } else
            if (x != 0 || y != 0) {
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = -(y + x+rx) / denominator;
                double backLeftPower = -(-y - x+rx) / denominator;
                double frontRightPower = (-y + x+rx) / denominator;
                double backRightPower = (y - x+rx) / denominator;
                motorFrontLeft.setPower(frontLeftPower);
                motorBackLeft.setPower(backLeftPower);
                motorFrontRight.setPower(frontRightPower);
                motorBackRight.setPower(backRightPower);
            } else {
                motorFrontLeft.setPower(-rx);
                motorBackLeft.setPower(-rx);
                motorFrontRight.setPower(-rx);
                motorBackRight.setPower(-rx);
            }
            int currentRaisedPosition = raise.getCurrentPosition();
            telemetry.addData("armPos", currentRaisedPosition);
            int raiseStep = 0;
            if(gamepad2.dpad_down) {
                raiseStep = 200;
            } else if (gamepad2.dpad_up) {
                raiseStep = -200;
            } else {
                raiseStep = 0;
            }
            int targetRaise = currentRaisedPosition;
            if (gamepad2.y) { // assuming bottom is 0, negative is up
                targetRaise = -1650;
            } else if (gamepad2.a) {
                targetRaise = -4120;
            } else if (gamepad2.b) {
                targetRaise = -2820;
            } else if (gamepad2.right_bumper){
                targetRaise = 0;
            } else {
                targetRaise = currentRaisedPosition + raiseStep;
            }
            raise.setTargetPosition(targetRaise);
            raise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            raise.setVelocity(ARM_FULL_SPEED_IN_COUNTS);
            while (raise.isBusy()) {
                y = gamepad1.left_stick_x;
                x = gamepad1.left_stick_y * 1.1; // Counteract imperfect strafing
                rx = gamepad1.right_stick_x;
                if (true) {
                    if (x != 0 || y != 0) {
                        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                        double frontLeftPower = -(y + x+rx) / denominator; // -1
                        double backLeftPower = -(-y - x+rx) / denominator; // 1
                        double frontRightPower = (-y + x+rx) / denominator; // -1
                        double backRightPower = (y - x+rx) / denominator; // 1
                        motorFrontLeft.setPower(frontLeftPower/2);
                        motorBackLeft.setPower(backLeftPower/2);
                        motorFrontRight.setPower(frontRightPower/2);
                        motorBackRight.setPower(backRightPower/2);
                    } else {
                        motorFrontLeft.setPower(-rx/2);
                        motorBackLeft.setPower(-rx/2);
                        motorFrontRight.setPower(-rx/2);
                        motorBackRight.setPower(-rx/2);
                    }
                }
                sleep(100);
            }
//            carousel.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

//            int currentArmPosition = arm.getCurrentPosition();
//            int armStep = 0;
//            if(gamepad2.dpad_down){
//                armStep = -200;
//            }else if(gamepad2.dpad_up){
//                armStep = 200;
//            }else{
//                armStep = (int)-gamepad2.left_stick_x*50;
//            }
//            int targetPosition = armStep + currentArmPosition;
//            // if(targetPosition < -10000){
//            //     targetPosition = -10000;
//            // }
//            // if(targetPosition > -20){
//            //     targetPosition = -20;
//            // }
//            arm.setTargetPosition(targetPosition);
//            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            arm.setVelocity(ARM_FULL_SPEED_IN_COUNTS);

            // if (gamepad2.dpad_down && currentArmPosition > -10000){
            //     arm.setPower(armPower);
            // }else if (gamepad2.dpad_up && currentArmPosition < -1200){
            //     arm.setPower(armPower);
            // }else{
            //     arm.setPower(0);
            // }
//            telemetry.addData("handPos", hand.getPosition());
            telemetry.addData("isOpen", isOpen);
            telemetry.addData("clawPos", claw.getPosition());
//            telemetry.addData("handPos", handPos);
//            telemetry.addData("clawPosition", claw.getPosition());
//            telemetry.addData("armPosition", currentArmPosition);
            telemetry.update();
            // if (armPower == 0) {
            //     double counterArmPower = 0;
            //     if(currentArmPosition >= ARM_VERTIAL_POSITION){
            //         counterArmPower = -0.12*Math.cos(0.5*Math.abs(currentArmPosition - ARM_FRONT_HORIZONTAL)*Math.PI/(ARM_FRONT_HORIZONTAL-ARM_VERTIAL_POSITION));
            //     }else if(currentArmPosition < ARM_VERTIAL_POSITION){
            //         counterArmPower = 0.16*Math.cos(0.5*Math.abs(currentArmPosition -  ARM_BACK_HORIZONTAL)*Math.PI/(ARM_VERTIAL_POSITION-ARM_BACK_HORIZONTAL));
            //     }
            //     arm.setPower(counterArmPower);

            // } else {
            //     arm.setPower(armPower);
            // }

//            if (gamepad2.x) {
//                clawPositive = !clawPositive;
//            }
//            if (clawPositive) {
//                claw.setPosition(0.7);
//            } else {
//                claw.setPosition(-0.6);
//            }

        }
    }
}
