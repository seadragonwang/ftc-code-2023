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
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    public static float HIGH_JUNCTION_IN_MILLIMETER = 850.9f;
    public static float MEDIUM_JUNCTION_IN_MILLIMETER = 596.9f;
    public static float LOW_JUNCTION_IN_MILLIMETER = 342.9f;

    // pulley circumference is 112 mm
    public static final double ARM_GEAR_RATIO = 13.7;
    public static final int ARM_MOTOR_SPEED_IN_RPM = 435;
    public static final double WHEEL_DIAMETER_MILLIMETTER = 96;
    public static final int WHEEL_MOTOR_SPPED_IN_RPM = 312;
    public static final double PULLEY_DIAMETER_IN_MM = 35.65;
    public static final double ARM_COUNTS_PER_MILLIMETER = (COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO)/(PULLEY_DIAMETER_IN_MM * Math.PI);
    public static final double ARM_FULL_SPEED_IN_COUNTS = COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO * ARM_MOTOR_SPEED_IN_RPM / 60;
    public static final int ARM_COUNTS_FOR_HIGH_JUNCTION = -(int) ((HIGH_JUNCTION_IN_MILLIMETER+50) * ARM_COUNTS_PER_MILLIMETER);
    public static final int ARM_COUNTS_FOR_MEDIUM_JUNCTION = -(int) ((MEDIUM_JUNCTION_IN_MILLIMETER+50) * ARM_COUNTS_PER_MILLIMETER);
    public static final int ARM_COUNTS_FOR_LOW_JUNCTION = -(int) ((LOW_JUNCTION_IN_MILLIMETER+50) * ARM_COUNTS_PER_MILLIMETER);
    private DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotorEx raise;
    Servo claw;
    Servo claw2;

    /**
     * Set up all motors and servos.
     */
    private void setup(){
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        raise = (DcMotorEx) hardwareMap.dcMotor.get("raise");
        claw = hardwareMap.servo.get("claw");
        claw2 = hardwareMap.servo.get("claw2");
    }

    /**
     * Power is positive, robot will slide left, otherwise slide right
     * @param power
     */
    private void slide(double power){
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(power);
        motorBackRight.setPower(-power);
    }

    /**
     * Power is positive, robot will move forward, otherwise move backward.
     * @param power
     */
    private void drive(double power){
        motorFrontLeft.setPower(-power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(power*1.05);
        motorBackRight.setPower(power*1.025);

    }

    private void operate(){
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y; // Counteract imperfect strafing
        if (gamepad2.x) {
            openClaw();
            sleep(50);
            raiseSlider(0);
        } else if (gamepad2.left_bumper) {
            closeClaw();
        }
        if (Math.abs(x) > Math.abs(y)) {
            y = 0;
        } else {
            x = 0;
        }
        double rx = gamepad1.right_stick_x;

        if(gamepad1.x){
            slide(0.5);
        }else if(gamepad1.b){
            slide(-0.5);
        }else if(gamepad1.y){
            drive(0.5);
        }else if(gamepad1.a){
            drive(-0.5);
        }else{
            drive(0);
        }
        if (x == 0 && y == 0) {
            if (rx != 0) {
                motorFrontLeft.setPower(-rx / 2);
                motorBackLeft.setPower(-rx / 2);
                motorFrontRight.setPower(-rx / 2);
                motorBackRight.setPower(-rx / 2);
            }
        } else if (x != 0) {
            slide(-x/2);
        } else {
            drive(-y/2);
        }
    }
    private void openClaw(){
        claw.setPosition(1);
        claw2.setPosition(0);
    }

    private void closeClaw(){
        claw.setPosition(0.65);
        claw2.setPosition(0.35);
    }

    private void raiseSlider(int targetPos){
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
                targetRaise = ARM_COUNTS_FOR_LOW_JUNCTION;
            } else if (gamepad2.a) {
                targetRaise = ARM_COUNTS_FOR_HIGH_JUNCTION;
            } else if (gamepad2.b) {
                targetRaise = ARM_COUNTS_FOR_MEDIUM_JUNCTION;
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
