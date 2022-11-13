//Don't touch!!!
//Tele2
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="OrcaSOLO",group="")
public class SoloTeleOrca extends LinearOpMode {
    public static final int CHASSIS_LENGTH_IN_MILLIMETER = 304;
    public static final int CHASSIC_WIDTH_IN_MILLIMETER = 352;
    public static final int FIELD_LENGTH_IN_MILLIMETER = 3590;
    public static final int SHIPPING_HUB_RADIUS_IN_MILLIMETER = 226;
    public static final int CAROUSAL_RADIUS_IN_MILLIMETER = 191;
    public static final int RABBIT_POSITION_MIN_IN_MILLIMETER = 700;
    public static final double TURN_DISTANCE_PER_DEGREE = 4.05878;
    public static final int RABBIT_POSITION_MAX_IN_MILLIMETER = 800;
    public static final int HALF_CHASSIS_WIDTH_IN_MILLIMETER = 114;
    public static final int HALF_CHASSIS_LENGTH_IN_MILLIMETER = 219;
    public static final int BLUE_SHIPPING_HUB_X_IN_MILLIMETER = 1187;
    public static final int BLUE_SHIPPING_HUB_Y_IN_MILLIMETER = 1543;
    public static final int RED_SHIPPING_HUB_X_IN_MILLIMETER = 2375;
    public static final int RED_SHIPPING_HUB_Y_IN_MILLIMETER = 1543;
    public static final int STORAGE_START_X_IN_MILLIMETER = 630;
    public static final int STORAGE_START_Y_IN_MILLIMETER = 0;
    public static final int STORAGE_END_X_IN_MILLIMETER = 1150;
    public static final int STORAGE_END_Y_IN_MILLIMETER = 630;
    public static final int STORAGE_CENTER_X_IN_MILLIMETER = 890;
    public static final int DISTANCE_FROM_CAROUSAL_SPINNER_TO_RIGHT_DISTANCE_SENSOR = 100;
    public static final int DISTANCE_FROM_CAROUSAL_SPINNER_TO_FRONT_DISTANCE_SENSOR = 100;
    public static final double ANGLE_RATIO = 1.4;
    public static final double SLIDE_RATIO = 1.2;
    public static final double CHASSIS_RADIUS = 203;
    public static final double COUNTS_PER_ENCODER_REV = 28;
    public static final double WHEEL_GEAR_RATIO = 19.203208556149733;
    //    public static final double ARM_GEAR_RATIO = 139.13824192336588;
    public static final double WHEEL_DIAMETER_MILLIMETTER = 96;
    public static final int WHEEL_MOTOR_SPPED_IN_RPM = 312;
    //    public static final int ARM_MOTOR_SPPED_IN_RPM = 43;
    public static final double WHEEL_COUNTS_PER_DEGREE = COUNTS_PER_ENCODER_REV * WHEEL_GEAR_RATIO / 360;
    public static final double WHEEL_FULL_SPEED_IN_COUNTS = COUNTS_PER_ENCODER_REV * WHEEL_GEAR_RATIO * WHEEL_MOTOR_SPPED_IN_RPM;
    //    public static final double ARM_COUNTS_PER_DEGREE = COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO / 360;
//    public static final double ARM_FULL_SPEED_IN_COUNTS = COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO * ARM_MOTOR_SPPED_IN_RPM;
    public static final double COUNTS_PER_MILLIMETER = (COUNTS_PER_ENCODER_REV * WHEEL_GEAR_RATIO) / (WHEEL_DIAMETER_MILLIMETTER * Math.PI);

//    public static final double COUNTS_PER_ENCODER_REV = 28;
    public static int ARM_VERTIAL_POSITION = 1109;
    public static int ARM_FRONT_HORIZONTAL = 2041;
    public static int ARM_BACK_HORIZONTAL = 177;
    // pulley circumference is 112 mm
    public static final double ARM_GEAR_RATIO = 19.2;
//    public static final double WHEEL_DIAMETER_MILLIMETTER = 96;
//    public static final int WHEEL_MOTOR_SPPED_IN_RPM = 312;
    public static final int ARM_MOTOR_SPPED_IN_RPM = 312;
    public static final double ARM_COUNTS_PER_DEGREE = COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO / 360;
    public static final double ARM_FULL_SPEED_IN_COUNTS = COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO * ARM_MOTOR_SPPED_IN_RPM;


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        DcMotorEx motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
        DcMotorEx motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        DcMotorEx motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("backRight");
        DcMotorEx raise = (DcMotorEx) hardwareMap.dcMotor.get("raise");
//        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
//        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distanceSensor;
        boolean runSlowMo = true;
//        DcMotor slider = hardwareMap.dcMotor.get("slider");
//        DcMotorEx arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
//        Servo hand = hardwareMap.servo.get("hand");
//        hand.setPosition(0.999);
//        boolean handUp = false;


//        Servo claw = hardwareMap.servo.get("claw");
//        hand.setPosition(0.36);
//        hand.setPosition(0.75);
//        hand.setDirection(Servo.Direction.REVERSE);
//        DcMotor carousel = hardwareMap.dcMotor.get("carousel");


//        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm"); // Minimum -2, Maximum -1272
        Servo claw = hardwareMap.servo.get("claw");

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
            if (gamepad1.x) {
                isOpen = !isOpen;
                if (isOpen) {
                    claw.setPosition(0.88);
                } else {
                    claw.setPosition(0.69);
                }
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
//            setDrivingMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double y = gamepad1.left_stick_x;
            double x = gamepad1.left_stick_y * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
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
            } else {
//            turn(180,0.1);
            }
            int currentRaisedPosition = raise.getCurrentPosition();
            telemetry.addData("armPos", currentRaisedPosition);
            int raiseStep = 0;
            if(gamepad1.dpad_down) {
                raiseStep = 200;
            } else if (gamepad1.dpad_up) {
                raiseStep = -200;
            } else {
                raiseStep = 0;
            }
            int targetRaise = currentRaisedPosition;
            if (gamepad1.y) { // assuming bottom is 0, negative is up
                targetRaise = -1650;
            } else if (gamepad1.a) {
                targetRaise = -4120;
            } else if (gamepad1.b) {
                targetRaise = -2820;
            } else if (gamepad1.right_bumper){
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
