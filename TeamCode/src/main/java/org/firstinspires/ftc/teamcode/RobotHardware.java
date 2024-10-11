// This file not meant for redistribution - copyright notice removed

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import  com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Hardware abstraction class for WA Robotics INTO THE DEEP competition robot
 */
public class RobotHardware {

    /*
     * Define constant values for interfacing with hardware devices (public so they CAN be used
     * by the calling OpMode
     */

    /* Drive constants */

    /* Odometry constants */
    public static final double DEADWHEEL_MM_PER_TICK = 0.0754; // MM per encoder tick (48MM diameter wheel @ 2000 ticks per revolution)
    private static final double DEADWHEEL_FORWARD_OFFSET = 138.55; //forward offset (length B) of aux deadwheel from robot center of rotation in MM
    private static final double DEADWHEEL_TRACKWIDTH = 332.51; // distance (length L) between left and right deadwheels in MM

    /* Arm (Viper-Slide) constants */


    /*
     * Member variables (private to hide from the calling opmode)
     */

    /* Hardware objects */
    private DcMotorEx leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;  //  Motors for Mecanum drive
    private DcMotor encoderRight, encoderLeft, encoderAux; // Encoders (deadwheels) for odometry

    /* Vision portal and AprilTag processor */
    private VisionPortal visionPortal; // Used to manage the video source.
    private AprilTagProcessor aprilTag; // Used for managing the AprilTag detection processor */

    private int currentRightCounter, currentLeftCounter, currentAuxCounter; // Odometry counters

    private Pose2D currentPosition; // Current robot vector (X,Y position and heading)

    private OpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    /* Constructor allows calling OpMode to pass a reference to itself. */
    public RobotHardware(OpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     */
    public void init()    {

        /* Define Mecanum drivetrain hardware instance variables */
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotorEx.class, "leftfront_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "rightfront_drive");
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotorEx.class, "leftback_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "rightback_drive");

        /* Set the run mode, braking, and direction for each */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // increased accuracy and balance from controls

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* Make sure robot it not moving */


        /* Update telemetry */
        myOpMode.telemetry.addData(">", "Hardware Initialized");
    }

    /* Basic movement methods for four-motor Mecanum drive train */
    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void move(double x, double y, double yaw, boolean sprint) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void updateOdometry() {

        /* save current encoder values */
        int oldRightCounter = currentRightCounter;
        int oldLeftCounter = currentLeftCounter;
        int oldAuxCounter = currentAuxCounter;

        /* read new encoder values */
        currentRightCounter = encoderRight.getCurrentPosition(); // check sign based on installation
        currentLeftCounter = encoderLeft.getCurrentPosition();
        currentAuxCounter = encoderAux.getCurrentPosition();

        /* calculate deltas (robot perspective) since last measurement */
        int dl = currentLeftCounter  - oldLeftCounter;
        int dr= currentRightCounter - oldRightCounter;
        int da = currentAuxCounter - oldAuxCounter;
        double dtheta = DEADWHEEL_MM_PER_TICK * (dr-dl) / DEADWHEEL_TRACKWIDTH;
        double dx = DEADWHEEL_MM_PER_TICK * (dl+dr) / 2.0;
        double dy = DEADWHEEL_MM_PER_TICK * (da - DEADWHEEL_FORWARD_OFFSET * (dr-dl) / DEADWHEEL_TRACKWIDTH);

        /* update the current position in field coordinate system from the deltas */
        double theta = currentPosition.getHeading(AngleUnit.RADIANS) + (dtheta / 2);
        double newX = currentPosition.getX(DistanceUnit.MM) + dx * Math.cos(theta) - dy * Math.sin(theta);
        double newY = currentPosition.getY(DistanceUnit.MM) + dx * Math.sin(theta) + dy * Math.cos(theta);
        double newHeading = (currentPosition.getHeading(AngleUnit.RADIANS) + dtheta) % (2.0 * Math.PI); // normalized to [0, 2pi)
        currentPosition = new Pose2D(DistanceUnit.MM, newX, newY, AngleUnit.RADIANS, newHeading);
    }

    /**
     * Return current X position in MM
     */
    public double getPosX() {
        return currentPosition.getX(DistanceUnit.MM);
    }

    /**
     * Return current X position in specified units
     */
    public double getPosX(DistanceUnit distanceUnit) {
        return currentPosition.getX(distanceUnit);
    }

    /**
     * Return current Y position in MM
     */
    public double getPosY() {
        return currentPosition.getY(DistanceUnit.MM);
    }

    /**
     * Return current Y position in specified units
     */
    public double getPosY(DistanceUnit distanceUnit) {
        return currentPosition.getY(distanceUnit);
    }

    /**
     * Return current Y position in MM
     */
    public double getHeading() {

        /* return a "rotational convention" angle from +X axis (-Pi, Pi], positive counter-clockwise */
        double theta = currentPosition.getHeading(AngleUnit.RADIANS);
        if(theta > Math.PI) {
            return -2 * Math.PI - theta;
        }
        else {
            return theta;
        }
    }

    /**
     * Return current heading in normalized RADIANS
     */
    public double getHeading(AngleUnit angleUnit) {
        /* return angle from +X axis (-Pi, Pi] or (-180, 180]  */
        if(angleUnit == AngleUnit.DEGREES) {
            double theta = currentPosition.getHeading(AngleUnit.DEGREES);
            if (theta > 180) {
                return -360 - theta;
            } else {
                return theta;
            }
        }
        else {
            double theta = currentPosition.getHeading(AngleUnit.RADIANS);
            if (theta > Math.PI) {
                return -2 * Math.PI - theta;
            } else {
                return theta;
            }
        }
    }
}
