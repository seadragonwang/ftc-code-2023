package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;

public abstract class OrcaRobot extends LinearOpMode {
    public static final int CHASSIS_LENGTH_IN_MILLIMETER = 312;
    public static final int CHASSIS_WIDTH_IN_MILLIMETER = 288;
    public static final int CHASSIS_DIAMETER_IN_MILLIMETER = (int)Math.sqrt(CHASSIS_LENGTH_IN_MILLIMETER*CHASSIS_LENGTH_IN_MILLIMETER+CHASSIS_WIDTH_IN_MILLIMETER*CHASSIS_WIDTH_IN_MILLIMETER);
    public static final double COUNTS_PER_ENCODER_REV = 28;

    public static float HIGH_JUNCTION_IN_MILLIMETER = 850.9f;
    public static float MEDIUM_JUNCTION_IN_MILLIMETER = 596.9f;
    public static float LOW_JUNCTION_IN_MILLIMETER = 342.9f;

    // pulley circumference is 112 mm
    public static final double WHEEL_DIAMETER_MILLIMETTER = 96;
    public static final int WHEEL_MOTOR_SPEED_IN_RPM = 312;
    public static final double WHEEL_GEAR_RATIO = 19.2;
    public static final double WHEEL_FULL_SPEED_IN_COUNTS = COUNTS_PER_ENCODER_REV * WHEEL_GEAR_RATIO * WHEEL_MOTOR_SPEED_IN_RPM / 60;
    public static final int WHEEL_COUNTS_PER_MILLIMETER = (int) ((COUNTS_PER_ENCODER_REV * WHEEL_GEAR_RATIO) / (WHEEL_DIAMETER_MILLIMETTER * Math.PI));
    public static final double WHEEL_DISTANCE_PER_DEGREE = 4.05878;
    public static final double ARM_GEAR_RATIO = 13.7;
    public static final int ARM_MOTOR_SPEED_IN_RPM = 1150;
    public static final double PULLEY_DIAMETER_IN_MM = 35.65;
    public static final int ARM_COUNTS_PER_MILLIMETER = (int) ((COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO) / (PULLEY_DIAMETER_IN_MM * Math.PI));
    public static final double ARM_FULL_SPEED_IN_COUNTS = COUNTS_PER_ENCODER_REV * ARM_GEAR_RATIO * ARM_MOTOR_SPEED_IN_RPM / 60;
    public static final int ARM_COUNTS_FOR_HIGH_JUNCTION = -(int) ((HIGH_JUNCTION_IN_MILLIMETER+125) * ARM_COUNTS_PER_MILLIMETER);
    public static final int ARM_COUNTS_FOR_MEDIUM_JUNCTION = -(int) ((MEDIUM_JUNCTION_IN_MILLIMETER+70) * ARM_COUNTS_PER_MILLIMETER);
    public static final int ARM_COUNTS_FOR_LOW_JUNCTION = -(int) ((LOW_JUNCTION_IN_MILLIMETER+50) * ARM_COUNTS_PER_MILLIMETER);
    public static final int ARM_COUNTS_FOR_FIVE_CONES = -(int) (135 * ARM_COUNTS_PER_MILLIMETER);
    public static final int ARM_COUNTS_FOR_FOUR_CONES = -(int) ((110) * ARM_COUNTS_PER_MILLIMETER);
    public static final int ARM_COUNTS_FOR_THREE_CONES = -(int) (77 * ARM_COUNTS_PER_MILLIMETER);
    public static final int ARM_COUNTS_FOR_TWO_CONES = -(int) (45 * ARM_COUNTS_PER_MILLIMETER);
    public static final int ARM_COUNTS_FOR_ONE_CONES = -(int) (12 * ARM_COUNTS_PER_MILLIMETER);
    protected DcMotorEx motorFrontLeft;
    protected DcMotorEx motorBackLeft;
    protected DcMotorEx motorFrontRight;
    protected DcMotorEx motorBackRight;
    protected DcMotorEx raise;
    protected Servo claw;
    protected Servo claw2;

    protected void openClaw(){
        claw.setPosition(1);
        claw2.setPosition(0);
    }

    protected void closeClaw(){
        claw.setPosition(0.65);
        claw2.setPosition(0.35);
    }

    /**
     * Set up all motors and servos.
     */
    protected void setup() {
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("backLeft");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("frontRight");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("backRight");
        raise = (DcMotorEx) hardwareMap.dcMotor.get("raise");
        claw = hardwareMap.servo.get("claw");
        claw2 = hardwareMap.servo.get("claw2");
    }

    public void setDrivingMotorMode(DcMotor.RunMode mode) {
        motorFrontRight.setMode(mode);
        motorFrontLeft.setMode(mode);
        motorBackRight.setMode(mode);
        motorBackLeft.setMode(mode);
    }
    public boolean isStillDriving() {
        return motorFrontLeft.isBusy() || motorFrontRight.isBusy() || motorBackLeft.isBusy() || motorBackRight.isBusy();
    }
}
