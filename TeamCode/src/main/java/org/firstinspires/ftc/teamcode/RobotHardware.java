// This file not meant for redistribution - copyright notice removed

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import  com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    /* Drive constants - allow OMNI-wheel drivetrain to operate in different at different, selectable "speeds" */
    public  static final double OMNI_MOTOR_POWER_LIMIT_NORMAL = 0.65; // Normal power limit
    public static final double OMNI_MOTOR_POWER_LIMIT_SPRINT = 1.0; // Sprint power limit
    public static final double OMNI_MOTOR_POWER_LIMIT_PRECISE = 0.4; // Precise positioning power limit

    /* PID Controller constants - for performing moveTo() and rotateTo() operations in autonomous driving */
    public static final double PID_CONTROLLER_X_KP = 0.1; // Proportional gain for X position
    public static final double PID_CONTROLLER_X_KI = 0.0; // Integral gain for X position
    public static final double PID_CONTROLLER_X_KD = 0.0; // Derivative gain for X position
    public static final double PID_CONTROLLER_Y_KP = 0.1; // Tolerance for Y position
    public static final double PID_CONTROLLER_Y_KI = 0.0; // Integral gain for Y position
    public static final double PID_CONTROLLER_Y_KD = 0.0; // Derivative gain for Y position
    public static final double PID_CONTROLLER_HEADING_KP = 0.1; // Proportional gain for heading
    public static final double PID_CONTROLLER_HEADING_KI = 0.0; // Integral gain for heading
    public static final double PID_CONTROLLER_HEADING_KD = 0.0; // Derivative gain for heading
    public static final double PID_POSITION_TOLERANCE = 10.0; // Tolerance for position in MM
    public static final double PID_HEADING_TOLERANCE = 0.0174533; // Tolerance for heading in radians (1 degree)


    /*
     * Odometry constants - these are used in the calculation of the current position (field coordinates).
     * The values are initially set from physical measurements of the robot but should be tweaked for accuracy
     * from testing (e.g., spin test and/or strafe/curve testing
     */
    public static final double DEADWHEEL_MM_PER_TICK = 0.0754; // MM per encoder tick (48MM diameter wheel @ 2000 ticks per revolution)
    public static final double DEADWHEEL_FORWARD_OFFSET = 138.55; //forward offset (length B) of aux deadwheel from robot center of rotation in MM
    public static final double DEADWHEEL_TRACKWIDTH = 332.51; // distance (length L) between left and right deadwheels in MM

    /* Arm (Viper-Slide) constants */

    /*
     * Member variables (private to hide from the calling opmode)
     */

    /* Hardware objects */
    private DcMotorEx leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;  //  Motors for Mecanum drive
    private DcMotor encoderRight, encoderLeft, encoderAux; // Encoders (deadwheels) for odometry
    private IMU imu; // IMU built into Rev Control Hub

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
        // call stop methods, set power to zero, initialize encoders, etc.

        /* Define odometry encoder hardware instance variables */
        encoderRight = myOpMode.hardwareMap.get(DcMotor.class, "encoder_right");
        encoderLeft = myOpMode.hardwareMap.get(DcMotor.class, "encoder_left");
        encoderAux = myOpMode.hardwareMap.get(DcMotor.class, "encoder_aux");

        /* Define IMU hardware instance variable */
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        /* Update telemetry */
        myOpMode.telemetry.addData(">", "Hardware Initialized");
    }

    /* Motion for four-motor Mecanum drive train */
    /**
    * Set Mecanum drivetrain motor powers
    */
    public void setMotorPowers(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Move robot according to robot-oriented axes motions
     * - Positive X is forward
     * - Positive Y is strafe left
     * - Positive Yaw is counter-clockwise
     */
    public void move(double x, double y, double yaw, boolean sprint) {
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

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
        setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
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
        double newHeading = (currentPosition.getHeading(AngleUnit.RADIANS) + dtheta);
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
     * Return current Y position in field coordinate system in MM units
     */
    public double getPosY() {
        return currentPosition.getY(DistanceUnit.MM);
    }

    /**
     * Return current Y position in field coordinate system in specified units
     */
    public double getPosY(DistanceUnit distanceUnit) {
        return currentPosition.getY(distanceUnit);
    }

    /**
     * Return normalized current pose heading in field coordinate system. Best for performing higher level
     * calculations and control.
     */
    public double getHeading() {
        return currentPosition.getHeading(AngleUnit.RADIANS) % (2.0 * Math.PI); // normalize to [0, 2pi);
    }

    /**
     * Return current heading in specified units as FTC "rotational convention" angle, i.e.,
     * (-180, 180] degrees or (-Pi, Pi] radians from +X axis - positive counter-clockwise.
     * Best for display in telemetry.
     */
    public double getHeading(AngleUnit angleUnit) {
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

    /**
     * Move robot to specified field coordinate position (X, Y) in MM units
     * This method should be called from a LinerOpMode and implements its own
     * loop to cover the robots motion to the specified position.
     */
    public void moveTo(double x, double y, double speed) {

        /* tracking variables for PID controller(s) */
        double integralSumX = 0.0;
        double integralSumY = 0.0;
        double lastErrorX = 0.0;
        double lastErrorY = 0.0;

        /* timer for time for move */
        ElapsedTime timer = new ElapsedTime();

        /* loop until position is obtained or opMode is stopped */
        while(((LinearOpMode)myOpMode).opModeIsActive()) {

            /* update the current position */
            updateOdometry();

            /* check if we are close enough to the target */
            if (Math.abs(x - getPosX()) < PID_POSITION_TOLERANCE && Math.abs(y - getPosY()) < PID_POSITION_TOLERANCE) {
                break;
            }

            /* calculate deltas of current position from target coordinates */
            double dx = x - getPosX();
            double dy = y - getPosX();

            /* convert deltas to robot-oriented X and Y errors for PID controller calculations */
            double h = getHeading();
            double errorX = dx * Math.cos(h) + dy * Math.sin(h);
            double errorY = -dx * Math.sin(h) + dy * Math.cos(h);

            /* use PID controller(s) to calculate robot-oriented X and Y components of motion */
            double derivativeX = (errorX - lastErrorX) / timer.seconds(); // rate of change of the error for X
            double derivativeY = (errorY - lastErrorY) / timer.seconds(); // rate of change of the error for Y
            integralSumX += errorX * timer.seconds(); // sum of all error over time for X
            integralSumY += errorY * timer.seconds(); // sum of all error over time for Y
            double powerX = PID_CONTROLLER_X_KP * errorX + PID_CONTROLLER_X_KI * integralSumX + PID_CONTROLLER_X_KD * derivativeX;
            double powerY = PID_CONTROLLER_Y_KP * errorY + PID_CONTROLLER_Y_KI * integralSumY + PID_CONTROLLER_Y_KD * derivativeY;

            /* set motor power through move method */
            move(powerX, powerY, 0.0, false); // no yaw

            /* update last error values */
            lastErrorX = errorX;
            lastErrorY = errorY;

            /* reset timer for next iteration */
            timer.reset();

            /* provide some time for motors to run */
            //((LinearOpMode) myOpMode).idle(); - not needed because opModeIsActve() will do this
        }
    }

    /**
     * Rotate robot to specified heading in field coordinate system
     * This method should be called from a LinerOpMode and implements its own
     * loop to cover the robots motion to the specified heading.
     */
    public void rotateTo(double heading, double speed) {

        /* tracking variables for PID controller */
        double integralSum = 0.0;
        double lastError = 0.0;

        /* timer for time for move */
        ElapsedTime timer = new ElapsedTime();

        /* loop until position is obtained or opMode is stopped */
        while(((LinearOpMode)myOpMode).opModeIsActive()) {

            /* update the current position */
            updateOdometry();

            /* check if we are close enough to the target */
            if (Math.abs(heading - getHeading()) < PID_HEADING_TOLERANCE) {
                break;
            }

            /* measure error between current heading and target for PID controller calculations */
            double error = heading - getHeading();

            /* use PID controller to calculate yaw rate (power) */
            double derivative = (error - lastError) / timer.seconds(); // rate of change of the error
            integralSum += error * timer.seconds(); // sum of all error over time
            double power = PID_CONTROLLER_HEADING_KP * error + PID_CONTROLLER_HEADING_KI * integralSum + PID_CONTROLLER_HEADING_KD * derivative;

            /* set motor power through move method */
            move(0.0, 0.0, power, false); // no x or y motion

            /* update last error values */
            lastError = error;

            /* reset timer for next iteration */
            timer.reset();
        }
    }

    /**
     * Move a relative distance from current position in robot-oriented coordinates
     * This method should be called from a LinerOpMode and implements its own
     * loop to cover the robots motion to the specified position.
     */
    public void moveRelative(double x, double y, double speed) {

    }

    /**
     * Rotate a relative angle from current heading in robot-oriented coordinates
     * This method should be called from a LinerOpMode and implements its own
     * loop to cover the robots motion to the specified heading.
     */
    public void rotateRelative(double heading, double speed) {

    }
}
