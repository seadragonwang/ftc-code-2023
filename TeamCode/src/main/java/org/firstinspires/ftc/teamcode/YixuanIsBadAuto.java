package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Auto (crish > yixen)")
public class YixuanIsBadAuto extends LinearOpMode {
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
    //    public DistanceSensor frontDistanceSensor;
//    public DistanceSensor leftDistanceSensor;
//    public DistanceSensor rightDistanceSensor;
    public DcMotorEx motorFrontLeft;
    public DcMotorEx motorFrontRight;
    public DcMotorEx motorBackLeft;
    public DcMotorEx motorBackRight;
    public Servo claw;
    public void setDrivingMotorMode(DcMotor.RunMode mode) {
        motorFrontRight.setMode(mode);
        motorFrontLeft.setMode(mode);
        motorBackRight.setMode(mode);
        motorBackLeft.setMode(mode);
    }
    public boolean isStillDriving() {
        return motorFrontLeft.isBusy() || motorFrontRight.isBusy() || motorBackLeft.isBusy() || motorBackRight.isBusy();
    }
    //    public void setDrivingMotorMode(DcMotor.RunMode mode) {
//        motorFrontRight.setMode(mode);
//        motorFrontLeft.setMode(mode);
//        motorBackRight.setMode(mode);
//        motorBackLeft.setMode(mode);
//    }
    protected void driveDistance(int distanceInMilliMeter, double speed) {
        if(distanceInMilliMeter == 0) return;
        int direction = distanceInMilliMeter / Math.abs(distanceInMilliMeter);
        int distanceInCounts = (int) (distanceInMilliMeter * COUNTS_PER_MILLIMETER);
        setDrivingMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setTargetPosition(-distanceInCounts);
        motorFrontLeft.setTargetPosition(distanceInCounts);
        motorBackRight.setTargetPosition(distanceInCounts);
        motorBackLeft.setTargetPosition(-distanceInCounts);
        setDrivingMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorFrontLeft.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackLeft.setVelocity(-WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);

        while (isStillDriving()) {
            sleep(100);
//            // telemetry.addData("Front right motor speed", motorFrontRight.getVelocity());
//            // telemetry.addData("Front left motor speed", motorFrontLeft.getVelocity());
//            // telemetry.addData("Back right speed", motorBackRight.getVelocity());
//            // telemetry.addData("Back left speed", motorBackLeft.getVelocity());
//            // telemetry.update()
        }
    }
    /*
    positive degrees turns CCW, negative degrees turns CW
     */
    protected void turn(int degrees, double speed) {
        if(degrees == 0) return;
        int direction = degrees / Math.abs(degrees);
        int distanceInCounts = (int) (degrees * TURN_DISTANCE_PER_DEGREE * 1.35 * COUNTS_PER_MILLIMETER);
        setDrivingMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setTargetPosition(distanceInCounts);
        motorFrontLeft.setTargetPosition(distanceInCounts);
        motorBackRight.setTargetPosition(distanceInCounts);
        motorBackLeft.setTargetPosition(distanceInCounts);
        setDrivingMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorFrontLeft.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackLeft.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);

        while (isStillDriving()) {
            sleep(100);
//            // telemetry.addData("Front right motor speed", motorFrontRight.getVelocity());
//            // telemetry.addData("Front left motor speed", motorFrontLeft.getVelocity());
//            // telemetry.addData("Back right speed", motorBackRight.getVelocity());
//            // telemetry.addData("Back left speed", motorBackLeft.getVelocity());
//            // telemetry.update()
        }
    }
    protected void slide(int distanceInMilliMeter, double speed) {
        if(distanceInMilliMeter == 0) return;
        int direction = distanceInMilliMeter / Math.abs(distanceInMilliMeter);
        int distanceInCounts = (int) (distanceInMilliMeter * COUNTS_PER_MILLIMETER);
        setDrivingMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setTargetPosition(-distanceInCounts);
        motorFrontLeft.setTargetPosition(-distanceInCounts);
        motorBackRight.setTargetPosition(distanceInCounts);
        motorBackLeft.setTargetPosition(distanceInCounts);
        setDrivingMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorFrontLeft.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackRight.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);
        motorBackLeft.setVelocity(WHEEL_FULL_SPEED_IN_COUNTS * speed * direction);

        while (isStillDriving()) {
            sleep(100);
//            // telemetry.addData("Front right motor speed", motorFrontRight.getVelocity());
//            // telemetry.addData("Front left motor speed", motorFrontLeft.getVelocity());
//            // telemetry.addData("Back right speed", motorBackRight.getVelocity());
//            // telemetry.addData("Back left speed", motorBackLeft.getVelocity());
//            // telemetry.update()
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Servo claw = hardwareMap.servo.get("claw");
        claw.setPosition(0.71);
        waitForStart();
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("backRight");

//        motorBackLeft.setPower(-0.42);
//        motorBackRight.setPower(-0.42);
//        motorFrontRight.setPower(0.42);
//        motorFrontLeft.setPower(-0.42);
//        sleep(1269);
//        motorBackLeft.setPower(-0);
//        motorBackRight.setPower(-0);
//        motorFrontRight.setPower(-0);
//        motorFrontLeft.setPower(-0);
        turn(-90, 0.05);
//        driveDistance(-350000000, 0.3);
    }
//    driveDistance
}