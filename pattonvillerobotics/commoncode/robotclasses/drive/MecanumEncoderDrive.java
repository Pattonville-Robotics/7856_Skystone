package org.pattonvillerobotics.commoncode.robotclasses.drive;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
//import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.pattonvillerobotics.commoncode.enums.Direction;

import java.util.Locale;

//import static org.apache.commons.math3.util.FastMath.PI;
//import static org.apache.commons.math3.util.FastMath.cos;
//import static org.apache.commons.math3.util.FastMath.sin;

/**
 * Created by greg on 10/2/2017.
 */

public class MecanumEncoderDrive {

    private static final String TAG = "MecanumEncoderDrive";
    private static final double COS_135 = Math.cos(3 * Math.PI / 4);
    private static final double SIN_135 = -COS_135;
    private static final double DEG_45 = Math.PI / 4;

    private static final String LEFT_REAR_MOTOR_NAME = "left_rear_motor";
    private static final String RIGHT_REAR_MOTOR_NAME = "right_rear_motor";
    public static final int TARGET_REACHED_THRESHOLD = 16;

    protected final RobotParameters robotParameters;
    public final LinearOpMode linearOpMode;
    public final HardwareMap hardwareMap;

    public final DcMotor leftDriveMotor, rightDriveMotor;
    public DcMotor leftRearMotor, rightRearMotor;
    private DcMotor.RunMode leftDriveSavedMotorMode, rightDriveSavedMotorMode;
    private DcMotor.RunMode leftRearSavedMotorMode, rightRearSavedMotorMode;

    public MecanumEncoderDrive(HardwareMap hardwareMap, LinearOpMode linearOpMode, RobotParameters robotParameters) {

        this.hardwareMap = hardwareMap;
        this.linearOpMode = linearOpMode;
        this.robotParameters = robotParameters;

        this.leftDriveMotor = hardwareMap.dcMotor.get("left_drive_motor");
        this.rightDriveMotor = hardwareMap.dcMotor.get("right_drive_motor");
        this.leftRearMotor = hardwareMap.dcMotor.get(LEFT_REAR_MOTOR_NAME);
        this.rightRearMotor = hardwareMap.dcMotor.get(RIGHT_REAR_MOTOR_NAME);

////        this.leftDriveMotor.setDirection(robotParameters.getLeftDriveMotorDirection());
        this.leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightDriveMotor.setDirection(robotParameters.getRightDriveMotorDirection());
        // rear motors use same direction as front drive motors
////        this.leftRearMotor.setDirection(robotParameters.getLeftDriveMotorDirection());
        this.leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightRearMotor.setDirection(robotParameters.getRightDriveMotorDirection());
telemetry("LFMotorDir: " + leftDriveMotor.getDirection());
telemetry("RFMotorDir: " + rightDriveMotor.getDirection());
    }

    /**
     * can be used to convert joystick values to polar
     *
     * @return coordinate array in the form of [r, theta]
     */
////    public static Vector2D toPolar(double x, double y) {
////        return new Vector2D(Math.hypot(x, y), Math.atan2(y, x));
////    }

    /**
     * used to drive a mecanum drive train
     *
     * @param angle    direction to go in radians
     * @param speed    speed to go
     * @param rotation rate of rotation
     */
    public void moveFreely(double angle, double speed, double rotation) {
        double xcomponent = COS_135 * (Math.cos(angle + DEG_45));
        double ycomponent = SIN_135 * (Math.sin(angle + DEG_45));


//        double scale = 1. / FastMath.max(FastMath.abs(xcomponent), FastMath.abs(ycomponent));
//        xcomponent *= scale;
//        ycomponent *= scale;

        this.leftDriveMotor.setPower((speed * ycomponent) - rotation);
        this.rightDriveMotor.setPower((speed * xcomponent) + rotation);
        this.leftRearMotor.setPower((speed * xcomponent) - rotation);
        this.rightRearMotor.setPower((speed * ycomponent) + rotation);
    }

    /**
     * drives a specific number of inches in a given direction
     *
     * @param direction the direction (forward or backward) to drive in
     * @param inches    the number of inches to drive
     * @param power     the power with which to drive
     */
    public void moveInches(Direction direction, double inches, double power) {
        //Move Specified Inches Using Motor Encoders

        int targetPositionLeft;
        int targetPositionRight;
        int targetPositionLeftRear;
        int targetPositionRightRear;

        Log.e(TAG, "Getting motor modes");
        storeMotorModes();

        resetMotorEncoders();

        int deltaPosition = (int) Math.round(inchesToTicks(inches));

        switch (direction) {
            case FORWARD: {
                targetPositionLeft = -deltaPosition;
                targetPositionRight = deltaPosition;
                targetPositionLeftRear = -deltaPosition;
                targetPositionRightRear = deltaPosition;
                break;
            }
            case BACKWARD: {
                targetPositionLeft = deltaPosition;
                targetPositionRight = -deltaPosition;
                targetPositionLeftRear = deltaPosition;
                targetPositionRightRear = -deltaPosition;
                break;
            }
            case LEFT: {
                targetPositionLeft = deltaPosition;
                targetPositionRight = deltaPosition;
                targetPositionLeftRear = -deltaPosition;
                targetPositionRightRear = -deltaPosition;
                break;
            }
            case RIGHT: {
                targetPositionLeft = -deltaPosition;
                targetPositionRight = -deltaPosition;
                targetPositionLeftRear = deltaPosition;
                targetPositionRightRear = deltaPosition;
                break;
            }
            default:
                throw new IllegalArgumentException("Direction must be Direction.FORWARDS, Direction.BACKWARDS, Direction.LEFT, or Direction.RIGHT!");
        }

        Log.e(TAG, "Setting motor power high");
        move(Direction.FORWARD, power); // To keep power in [0.0, 1.0]. Encoders control direction

        Log.e(TAG, "Setting target position");
        setMotorTargets(targetPositionLeft, targetPositionRight, targetPositionLeftRear, targetPositionRightRear);

        Log.e(TAG, "Setting motor modes");
        setMotorsRunToPosition();

telemetry("LFMotorDir: " + leftDriveMotor.getDirection());
telemetry("RFMotorDir: " + rightDriveMotor.getDirection());
        telemetry("Moving " + inches + " inches at power " + power);
////        telemetry("LFMotorT: " + targetPositionLeft);
        telemetry("LFMotorT: " + leftDriveMotor.getTargetPosition());
        telemetry("RFMotorT: " + targetPositionRight);
        telemetry("LeftPower: " + leftDriveMotor.getPower());
        telemetry("RightPower: " + rightDriveMotor.getPower());
        telemetry("EncoderDelta: " + deltaPosition);
        Telemetry.Item distance = telemetry("DistanceL: N/A DistanceR: N/A");
        Telemetry.Item distanceRear = telemetry("DistanceLR: N/A DistanceRR: N/A");

        while (isMovingToPosition()
                || !motorsReachedTarget(targetPositionLeft, targetPositionRight, targetPositionLeftRear, targetPositionRightRear)
                && linearOpMode.opModeIsActive()) {
            distance.setValue(String.format(Locale.getDefault(), "DistanceL: %d DistanceR: %d", leftDriveMotor.getCurrentPosition(), rightDriveMotor.getCurrentPosition()));
            distanceRear.setValue(String.format(Locale.getDefault(), "DistanceLR: %d DistanceRR: %d", leftRearMotor.getCurrentPosition(), rightRearMotor.getCurrentPosition()));
            linearOpMode.telemetry.update();
        }
        Log.e(TAG, "Setting motor power low");
        stop();

        Log.e(TAG, "Restoring motor mode");
        restoreMotorModes();

        sleep(100);
    }

    public void rotateDegrees(Direction direction, double degrees, double speed) {
        //Move specified degrees using motor encoders
        //TODO: use the IMU on the REV module for more accurate turns
        int targetPositionLeft;
        int targetPositionRight;
        int targetPositionLeftRear;
        int targetPositionRightRear;

        Log.e(TAG, "Getting motor modes");
        storeMotorModes();

        resetMotorEncoders();

        double inches = degreesToInches(degrees);
        int deltaPosition = (int) Math.round(inchesToTicks(inches));

        switch (direction) {
            case COUNTERCLOCKWISE: {
                targetPositionLeft = deltaPosition;
                targetPositionRight = deltaPosition;
                targetPositionLeftRear = deltaPosition;
                targetPositionRightRear = deltaPosition;
                break;
            }
            case CLOCKWISE: {
                targetPositionLeft = -deltaPosition;
                targetPositionRight = -deltaPosition;
                targetPositionLeftRear = -deltaPosition;
                targetPositionRightRear = -deltaPosition;
                break;
            }
            default:
                throw new IllegalArgumentException("Direction must be Direction.CLOCKWISE or Direction.COUNTERCLOCKWISE!");
        }

        setMotorTargets(targetPositionLeft, targetPositionRight);

        Log.e(TAG, "Setting motor modes");
        setMotorsRunToPosition();

        Telemetry.Item[] items = new Telemetry.Item[]{
                telemetry("Rotating " + degrees + " degrees at speed " + speed).setRetained(true),
                telemetry("LFMotorT: " + targetPositionLeft).setRetained(true),
                telemetry("RFMotorT: " + targetPositionRight).setRetained(true),
                telemetry("LRMotorT: " + targetPositionLeftRear).setRetained(true),
                telemetry("RRMotorT: " + targetPositionRightRear).setRetained(true),
                telemetry("EncoderDelta: " + deltaPosition).setRetained(true),
                telemetry("DistanceLF: DistanceRF:").setRetained(true),
                telemetry("DistanceLR: DistanceRR:").setRetained(true)
        };
        Telemetry.Item distance = items[6];
        Telemetry.Item distanceRear = items[7];

        move(Direction.FORWARD, speed); // To keep speed in [0.0, 1.0]. Encoders control direction
        while (isMovingToPosition()
                || !motorsReachedTarget(targetPositionLeft, targetPositionRight, targetPositionLeftRear, targetPositionRightRear)
                && linearOpMode.opModeIsActive()) {
            distance.setValue("DistanceLF: " + leftDriveMotor.getCurrentPosition() + " DistanceRF: " + rightDriveMotor.getCurrentPosition());
            distanceRear.setValue("DistanceLR: " + leftRearMotor.getCurrentPosition() + " DistanceRR: " + rightRearMotor.getCurrentPosition());
            linearOpMode.telemetry.update();
        }
        stop();

        Log.e(TAG, "Restoring motor mode");
        restoreMotorModes();

        for (Telemetry.Item i : items)
            i.setRetained(false);

        sleep(100);
    }

    public void move(Direction direction, double power) {
        double angle;

        switch (direction) {
            case FORWARD:
                angle = Math.toRadians(90);
                break;
            case BACKWARD:
                angle = Math.toRadians(270);
                break;
            case LEFT:
                angle = Math.toRadians(180);
                break;
            case RIGHT:
                angle = 0;
                break;
            default:
                throw new IllegalArgumentException("Direction must be FORWARD, BACKWARD, LEFT, or RIGHT");
        }
        moveFreely(angle, power, 0);
    }

    public void turn(Direction direction, double power) {
        double rotation;

        switch (direction) {
            case LEFT:
                rotation = -power;
                break;
            case RIGHT:
                rotation = power;
                break;
            default:
                throw new IllegalArgumentException("Direction must be LEFT or RIGHT");
        }
        moveFreely(0, 0, rotation);
    }

    public double degreesToInches(double degrees) {
        return robotParameters.getWheelBaseCircumference() * degrees / 360;
    }

    public double inchesToTicks(double inches) {
        return robotParameters.getAdjustedTicksPerRevolution() * inches / robotParameters.getWheelCircumference();
    }

    protected boolean isMovingToPosition() {
        return leftDriveMotor.isBusy() || rightDriveMotor.isBusy() ||
               leftRearMotor.isBusy()  || rightRearMotor.isBusy();
    }

    protected void resetMotorEncoders() {
        leftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    protected void restoreMotorModes() {
        leftDriveMotor.setMode(leftDriveSavedMotorMode);
        rightDriveMotor.setMode(rightDriveSavedMotorMode);
        leftRearMotor.setMode(leftRearSavedMotorMode);
        rightRearMotor.setMode(rightRearSavedMotorMode);
    }

    protected void setMotorsRunToPosition() {
        if (leftDriveMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (rightDriveMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (leftRearMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            leftRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (rightRearMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            rightRearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void sleep(long milli) {
        this.linearOpMode.sleep(milli);
    }

    public void stop() {
        moveFreely(0, 0, 0);
    }

    protected void storeMotorModes() {
        leftDriveSavedMotorMode = leftDriveMotor.getMode();
        rightDriveSavedMotorMode = rightDriveMotor.getMode();
        leftRearSavedMotorMode = leftRearMotor.getMode();
        rightRearSavedMotorMode = rightRearMotor.getMode();
    }

    protected void setMotorTargets(final int targetPositionLeft, final int targetPositionRight) {
        // set front and rear motors to same target
        setMotorTargets(targetPositionLeft, targetPositionRight, targetPositionLeft, targetPositionRight);
    }

    protected void setMotorTargets(int targetPositionLeft, int targetPositionRight, int targetPositionLeftRear, int targetPositionRightRear) {
        leftDriveMotor.setTargetPosition(targetPositionLeft);
        rightDriveMotor.setTargetPosition(targetPositionRight);
        leftRearMotor.setTargetPosition(targetPositionLeftRear);
        rightRearMotor.setTargetPosition(targetPositionRightRear);
    }

    protected boolean motorsReachedTarget(int targetPositionLeft, int targetPositionRight, int targetPositionLeftRear, int targetPositionRightRear) {
        return reachedTarget(leftDriveMotor.getCurrentPosition(), targetPositionLeft, rightDriveMotor.getCurrentPosition(), targetPositionRight) &&
                reachedTarget(leftRearMotor.getCurrentPosition(), targetPositionLeftRear, rightRearMotor.getCurrentPosition(), targetPositionRightRear);
    }

    protected boolean reachedTarget(int currentPositionLeft, int targetPositionLeft, int currentPositionRight, int targetPositionRight) {
        return Math.abs(currentPositionLeft - targetPositionLeft) < TARGET_REACHED_THRESHOLD && Math.abs(currentPositionRight - targetPositionRight) < TARGET_REACHED_THRESHOLD;
    }

    public Telemetry.Item telemetry(String message) {
        return this.linearOpMode.telemetry.addData("EncoderDrive", message);
    }
}
