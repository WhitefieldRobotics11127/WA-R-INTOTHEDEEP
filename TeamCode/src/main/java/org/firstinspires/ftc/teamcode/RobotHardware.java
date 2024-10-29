// This file not meant for redistribution - copyright notice removed
/*
 * This class provides a hardware abstraction layer for WA Robotics INTO THE DEEP competition robot.
 *
 * This RobotHardware class is modified from the FTC SDK example code and was rebuilt from the
 * ground up in the 2024-2025 season in conjunction with a new crop of students and mentors.
 *
 * Support for Mecanum (Omni) drivetrain, odometry, IMU, and vision processing is included, which
 * can (hopefully) be reused from year to year. Support for INTO THE DEEP game-specific hardware,
 * such as the Viper-Slide arm and the servos for the claw are season-specific but can serve as
 * examples , will be added as needed.
 *
 * Also included in this class are methods and classes for performing autonomous motion using
 * odometry. Multiple options (direct drive, strafe, and rotate commands; relative X, Y, and heading
 * changes; and move and rotate to absolute field coordinates) are included.
 *
 * Many parameter values must be tuned for the specific robot and competition, and these are noted
 * in the comments.
 *
 * To simplify all calculations, all lengths are in MM and all angles are in radians. The FTC
 * coordinate system is used, with the +X-axis forward, the +Y-axis to the left, the +Z-axis up,
 * and +Yaw is counterclockwise.
 */

package org.firstinspires.ftc.teamcode;

/*
 * For the most part, imports are managed by the Android Studio IDE through the "Auto Import"
 * feature setting (under File->Settings, drill down to Editor>General>Auto Import), but may
 * occasionally need to be cleaned up to remove unused imports.
 */
import static com.qualcomm.robotcore.util.Range.clip;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Hardware abstraction class for WA Robotics INTO THE DEEP competition robot
 */
public class RobotHardware {

    /* ----- Public constants (so they CAN be used the calling OpMode - just in case) ----- */

    /* Parameter values for drivetrain motors.
     * Even though all robots will likely use four-motor Mecanum (Omni) drivetrains, these constants
     * may have to be modified each year based on the robots mechanical configuration, weight, and
     * performance
     */
    
    // Allow drivetrain to operate in different at different, selectable "speeds"
    public  static final double MOTOR_SPEED_FACTOR_NORMAL = 0.65; // Normal power limit
    public static final double MOTOR_SPEED_FACTOR_DAVIS = 1.0; // Sprint power limit
    public static final double MOTOR_SPEED_FACTOR_PRECISE = 0.4; // Precise positioning power limit
    
    // Correction factors for individual motors to account for mechanical differences
    // NOTE: If the robot is not driving straight, adjust these values to correct the issue.
    // NOTE: these values may not be needed if all motors are using encoders and the run modes
    // are set to RUN_USING_ENCODER
    public static final double OMNI_CORRECTION_LEFT_FRONT = 1.0; // Correction factor for left front motor
    public static final double OMNI_CORRECTION_RIGHT_FRONT = 1.0; // Correction factor for right front motor
    public static final double OMNI_CORRECTION_LEFT_BACK = 1.0; // Correction factor for left back motor
    public static final double OMNI_CORRECTION_RIGHT_BACK = 1.0; // Correction factor for right back motor

    /* Parameter values for odometry calculations.
     * These are used in the calculation of the current position (relative movement and field
     * coordinates). The values are initially set from physical measurements of the robot but should
     * be tweaked for accuracy from testing (e.g., spin test and/or strafe/curve testing).
     */
    public static final int DEADWHEEL_LEFT_DIRECTION = 1; // Allows for adjustment of + direction of left encoder - should be installed front to back
    public static final int DEADWHEEL_RIGHT_DIRECTION = -1; // Allows for adjustment of + direction of right encoder - should be installed front to back
    public static final int DEADWHEEL_AUX_DIRECTION = 1; // Allows for adjustment of + direction of aux encoder - should be installed left to right
    public static final double DEADWHEEL_MM_PER_TICK = 0.0754; // MM per encoder tick (48MM diameter wheel @ 2000 ticks per revolution)
    public static final double DEADWHEEL_FORWARD_OFFSET = -106.0; //forward offset (length B) of aux deadwheel from robot center of rotation in MM (negative if behind)
    public static final double DEADWHEEL_TRACKWIDTH = 305.0; // distance (length L) between left and right deadwheels in MM

    /* Parameter values for arm (Viper-Slide).
     * Put any parameter values here, e.g. max and min positions for extension, etc.
     */

    /* Parameter values for claw
     * Put any parameter values here, e.g. servo position for open and close, etc. ******
     */

    /* Constants for autonomous motion routines.
     * These may require a lot of tweaking.
     */

    // Tolerance values for closed-loop controllers for use in translate and rotate commands
    public static final double MOVE_POSITION_TOLERANCE = 10.0; // Tolerance for position in MM
    public static final double ROTATE_HEADING_TOLERANCE = 0.03489; // Tolerance for heading in radians (~2 degrees)

    public static final double PID_CONTROLLER_X_DEADBAND = 5.0; // Deadband range for X power calculation. Should be less than MOVE_POSITION_TOLERANCE
    public static final double PID_CONTROLLER_Y_DEADBAND = 5.0; // Deadband range for Y power calculation. Should be less than MOVE_POSITION_TOLERANCE
    public static final double PID_CONTROLLER_YAW_DEADBAND = 0.02; // Deadband range for Yaw power calculation. Should be less than ROTATE_HEADING_TOLERANCE


    // PID gain values for each of the three closed-loop controllers (X, Y, and heading)
    public static final double PID_CONTROLLER_X_KP = 0.16; // Proportional gain for X position error
    public static final double PID_CONTROLLER_X_KI = 0.0; // Integral gain for X position error
    public static final double PID_CONTROLLER_X_KD = 0.0; // Derivative gain for X position error
    public static final double PID_CONTROLLER_Y_KP = 0.1; // Tolerance for Y position error
    public static final double PID_CONTROLLER_Y_KI = 0.0; // Integral gain for Y position error
    public static final double PID_CONTROLLER_Y_KD = 0.0; // Derivative gain for Y position error
    public static final double PID_CONTROLLER_YAW_KP = 0.1; // Proportional gain for heading error
    public static final double PID_CONTROLLER_YAW_KI = 0.0; // Integral gain for heading error
    public static final double PID_CONTROLLER_YAW_KD = 0.0; // Derivative gain for heading error
    
    /* ----- Member variables (private to hide from the calling opmode) ----- */

    /*
     * Hardware objects for current robot hardware.
     * Any functionality or properties of any of these objects needed by opmodes will need to be
     * exposed through methods added to this class (thus the "abstraction" layer).
     */
    /*
     * NOTE: We should use the DCMotorEx class for all motors connected to a REV Control Hub or
     * REV Expansion whether or not  we are using RUN_USING_ENCODERS or other extended functionality
     * because the built-in REV motor controllers support all the functionality of the DCMotorEx
     * class.
     */
    private DcMotorEx leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;  //  Motors for Mecanum drive
    private DcMotorEx encoderRight, encoderLeft, encoderAux; // Encoders (deadwheels) for odometry
    private IMU imu; // IMU built into Rev Control Hub

    /*
     * NOTE: Some of these objects may be pointed to the same motor/encoder ports as the deadwheel
     * encoders used for odometry. Any motor that shares a port with a deadwheel encoder should be
     * set to RUN_WITHOUT_ENCODER mode
     */
    //private DcMotorEx armRotation, armExtension; // Motors for Viper-Slide arm extension and rotation
    //private Servo clawServo; // Servo for claw open/close

    //private VisionPortal visionPortal; // Used to manage the video source.
    //private AprilTagProcessor aprilTag; // Used for managing the AprilTag detection processor

    /*
     * Variables for tracking robot state     
     */
    // last read odometry deadwheel encoder positions 
    // NOTE: these are used to calculate encoder deltas since last call to updateOdometry()
    // These are made public temporarily for initial testing/tuning purposes
    public int lastRightEncoderPosition, lastLeftEncoderPosition, lastAuxEncoderPosition;

    // translated x, y, and heading odometry counters in mm since last reset
    // NOTE: these are updated by the updateOdometry() method and used for simple movement commands
    // (drive, strafe, rotate).
    private double xOdometryCounter, yOdometryCounter, headingOdometryCounter;
    
    // Current robot position (x,y, heading) in field coordinate system
    // NOTE: this is updated by the updateOdometry() method and used for translation and/or rotation
    // to field coordinates
    private Pose2D currentFieldPosition;

    // keep a reference to the calling opmode so that we have access to hardwareMap and other
    // properties and statuses from the running opmode.
    private OpMode myOpMode = null; 

    /**
     * Constructor allows calling OpMode to pass a reference to itself.
     * @param opmode the OpMode that is creating this RobotHardware instance
     */
    public RobotHardware(OpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     */
    public void init()    {

        // Define Mecanum drivetrain hardware instance variables
        leftFrontDrive  = myOpMode.hardwareMap.get(DcMotorEx.class, "leftfront_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "rightfront_drive");
        leftBackDrive  = myOpMode.hardwareMap.get(DcMotorEx.class, "leftback_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotorEx.class, "rightback_drive");

        // Set the direction, braking, and run mode for each motor 
        leftFrontDrive.setDirection(DcMotorEx.Direction.REVERSE); // based on which way the motor is mounted
        leftFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // don't allow overrun
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // increased accuracy and balance from controls

        leftBackDrive.setDirection(DcMotorEx.Direction.REVERSE);
        leftBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightBackDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Make sure robot it not moving (power to zero and/or stop commands), and initialize
        // encoders
        // NOTE: There's a lot of ambiguity in the documentation and the forums of whether the motor
        // power will be set to 0 when setting the mode to STOP_AND_RESET_ENCODER and whether the
        // mode will change back to RUN_USING_ENCODER after the encoders are reset or not. For both
        // these questions, the the documentation basically says "some motor controllers yes, some
        // motor controllers no." Also, there are questions of whether a short sleep by is needed
        // (e.g., 100ms) after setting the mode to STOP_AND_RESET_ENCODER to allow the encoders to
        // reset before setting the mode back to RUN_USING ENCODER.
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setPower(0.0);
        
        // ????? What do we need here (if anything) to ensure that the encoders are reset before we
        // set the run mode back to RUN_USING_ENCODER? A Thread.sleep() call, a myOpMode.wait()
        // call, a myOpMode.idle() call (only for Linear opmodes), etc.?
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // may not be needed
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Define odometry encoder hardware instance variables
        encoderRight = myOpMode.hardwareMap.get(DcMotorEx.class, "encoder_right");
        encoderLeft = myOpMode.hardwareMap.get(DcMotorEx.class, "encoder_left");
        encoderAux = myOpMode.hardwareMap.get(DcMotorEx.class, "encoder_aux");

        // Reset the encoder values
        encoderRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoderLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoderAux.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Define IMU hardware instance variable
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        // Define arm and claw hardware instance variables
        //armRotation = myOpMode.hardwareMap.get(DcMotorEx.class, "arm_rotation");
        //armExtensions = myOpMode.hardwareMap.get(DcMotorEx.class, "arm_extension");
        //clawServo = myOpMode.hardwareMap.get(Servo.class, "claw_servo");
    }

    /* ----- Low level motion methods for four-motor Mecanum drive train ----- */

    /**
    * Set Mecanum drivetrain motor powers
    */
    public void setMotorPowers(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {

        /*
         * NOTE: since we are using encoders on all wheels, what we are really doing here is
         * specifying a ratio (0.0 to 1.0) of the maximum RPM speed of the motor. Similar to
         * calling setVelocity() but without having to know the RPM ranges and values. The motor
         * controller code (DcMotorEx class) can then take care of changing voltage levels from the
         * battery and variations in friction in the motors for weight distribution of the robot to
         * provide more balanced speed control to the four wheels. The actual speed of the motor is
         * managed by the DcMotorEx class utilizing a built-in PID controller to attain the
         * calculated RPM. We may want to adjust the PID controller gain values via methods in the
         * DcMotorEx class to obtain stable operation.
         */

        // Send powers to the wheels and apply corrections.
        leftFrontDrive.setPower(leftFrontPower * OMNI_CORRECTION_LEFT_FRONT);
        rightFrontDrive.setPower(rightFrontPower* OMNI_CORRECTION_RIGHT_FRONT);
        leftBackDrive.setPower(leftBackPower * OMNI_CORRECTION_LEFT_BACK);
        rightBackDrive.setPower(rightBackPower * OMNI_CORRECTION_RIGHT_BACK);
    }

    /**
     * Drive robot according to robot-oriented axes of motion
     * This method can be used by teleop opmodes directly to drive the robot, since the human on
     * the gamepad will be viewing and controlling the robot on the field with subtle adjustments
     * (thus "closed-loop" controller), as well as by the higher-level motion routines for
     * autonomous driving.
     * @param x "power" (relative speed) for axial movement (+ is forward)
     * @param y power for lateral movement (strafe) (+ is left)
     * @param yaw power for rotation (+ is counter-clockwise)
     */
    public void move(double x, double y, double yaw, double speed) {

        // Calculate the powers for the four motors attached to the mecanum wheels based on the
        // specified x, y, yaw powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0 but retain the balance between the four
        // wheels calculated above.
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        
        // adjust normalized powers by the speed factor and set motor powers
        setMotorPowers(
                leftFrontPower * speed,
                rightFrontPower * speed,
                leftBackPower * speed,
                rightBackPower * speed
        );
    }

    /**
     * Stop robot motion
     */
    public void stop() {

        // Set all motor powers to zero
        setMotorPowers(0.0, 0.0, 0.0, 0.0);
    }

    /* ----- Methods for three-wheel odometry ----- */

    /**
     * Read odometry wheel encoders and update the current odometry counters and field position
     * of the robot.
     * This method should be called at the beginning of each loop in an autonomous opMode and in any
     * subloops in translation or rotation routines.
     */
    public void updateOdometry() {

        /*
         * NOTE: This code is adapted from the discussion of odometry in Game Manual 0 found here:
         * https://gm0.org/en/latest/docs/software/concepts/odometry.html. The code currently
         * implements linear approximations of deltas, i.e., assuming that the individual x and y
         * movements between updates occurred in straight lines. This may be fine for the mid-level
         * autonomous movement functions (drive, strafe, turn), but may build up errors over time
         * with movement functions or user driving. It would be more accurate if we use differential
         * equations (referred to as "Pose Exponentials" in the GM0 documentation) for computing the
         * deltas, i.e., assume that the movement between updates occurs in arcs. The differential
         * equation calculations converge to zero error faster than the linear approximations as the
         * frequency of odometry updates increases, i.e., loop times decrease.
         */

        // save current encoder values
        int oldRightCounter = lastRightEncoderPosition;
        int oldLeftCounter = lastLeftEncoderPosition;
        int oldAuxOdometryCounter = lastAuxEncoderPosition;

        // read new encoder values from odometry deadwheels and adjust for direction
        lastRightEncoderPosition = encoderRight.getCurrentPosition() * DEADWHEEL_RIGHT_DIRECTION;
        lastLeftEncoderPosition = encoderLeft.getCurrentPosition() * DEADWHEEL_LEFT_DIRECTION;
        lastAuxEncoderPosition = encoderAux.getCurrentPosition() * DEADWHEEL_AUX_DIRECTION;

        // calculate x, y, and theta (heading) deltas (robot perspective) since last measurement
        int dl = lastLeftEncoderPosition  - oldLeftCounter;
        int dr= lastRightEncoderPosition - oldRightCounter;
        int da = lastAuxEncoderPosition - oldAuxOdometryCounter;
        //double dtheta = DEADWHEEL_MM_PER_TICK * (dr-dl) / DEADWHEEL_TRACKWIDTH; // this approximation seems like it would build up a lot of error
        double dtheta = Math.acos(1 - Math.pow(DEADWHEEL_MM_PER_TICK * (dr -dl), 2) / (2 * Math.pow(DEADWHEEL_TRACKWIDTH, 2))); // should this be arctan?
        double dx = DEADWHEEL_MM_PER_TICK * (dl+dr) / 2.0;
        double dy = DEADWHEEL_MM_PER_TICK * da - DEADWHEEL_FORWARD_OFFSET * dtheta;

        // update the x, y, and heading odometry counters
        xOdometryCounter += dx;
        yOdometryCounter += dy;
        headingOdometryCounter += dtheta;
        
        // update the current position in field coordinate system from the deltas
        double theta = currentFieldPosition.getHeading(AngleUnit.RADIANS) + (dtheta / 2);
        double newX = currentFieldPosition.getX(DistanceUnit.MM) + dx * Math.cos(theta) - dy * Math.sin(theta);
        double newY = currentFieldPosition.getY(DistanceUnit.MM) + dx * Math.sin(theta) + dy * Math.cos(theta);
        double newHeading = (currentFieldPosition.getHeading(AngleUnit.RADIANS) + dtheta);
        currentFieldPosition = new Pose2D(DistanceUnit.MM, newX, newY, AngleUnit.RADIANS, newHeading);
    }

    /**
     * Reset the x, y, and heading odometry counters to zero.
     * This method should be called at the beginning of simple move and rotate commands to ensure
     * that translation and rotation are relevant to robot's starting position
     */
    public void resetOdometryCounters() {

        // reset the odometry counters to zero
        xOdometryCounter = 0.0;
        yOdometryCounter = 0.0;
        headingOdometryCounter = 0.0;
    }

    /**
     * Return axial (x) odometry counter in MM units. This method is primarily for retrieveal of the
     * odometry counters by the opmode for display in telemetry during testing.
     */
    public double getOdometryX() {
        return xOdometryCounter;
    }

    /**
     * Return lateral (y) odometry counter in MM units. This method is primarily for retrieveal of
     * the odometry counters by the opmode for display in telemetry during testing.
     */
    public double getOdometryY() {
        return yOdometryCounter;
    }

    /**
     * Return heading odometry counter in degrees. This method is primarily for retrieveal of the
     * odometry counters by the opmode for display in telemetry during testing.
     */
    public double getOdometryHeading() {
        double theta = headingOdometryCounter / (2 * Math.PI) * 360;
        if (theta > 180) {
            return -360 - theta;
        } else {
            return theta;
        }
    }

    /**
     * Set the robot's position in the field coordinate system in MM and radians.
     * This method should be called to initialize the robot's initial position from known starting
     * position (e.g., from the field setup or from a known position on the field). This method may
     * also be called when updating the robot's position from vision processing or other sensors.
     * @param x x-coordinate of center of robot in field coordinates
     * @param y y-coordinate of center of robot in field coordinates
     * @param heading current angle of robot relative to positive x-axis in field coordinates
     */
    public void setFieldPosition(double x, double y, double heading) {
        currentFieldPosition = new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, heading);
    }

    /**
     * Set the robot's position in the field coordinate system in specified distance and angle
     * units.
     * @param x x-coordinate of center of robot in field coordinates
     * @param y y-coordinate of center of robot in field coordinates
     * @param heading current angle of robot relative to positive x-axis in field coordinates
     */
    public void setFieldPosition(double x, double y, DistanceUnit dUnit, double heading, AngleUnit aUnit) {
        currentFieldPosition = new Pose2D(dUnit, x, y, aUnit, heading);
    }

    /**
     * Return current field position as a Pose2D object
     */
    public Pose2D getCurrentFieldPosition() {
        return currentFieldPosition;
    }

    /**
     * Return X coordinate of current field position in MM units
     */
    public double getPosX() {
        return currentFieldPosition.getX(DistanceUnit.MM);
    }

    /**
     * Return X coordinate of current field position in specified units
     */
    public double getPosX(DistanceUnit distanceUnit) {
        return currentFieldPosition.getX(distanceUnit);
    }

    /**
     * Return Y coordinate of current field position in MM units
     */
    public double getPosY() {
        return currentFieldPosition.getY(DistanceUnit.MM);
    }

    /**
     * Return Y coordinate of current field position in specified units
     */
    public double getPosY(DistanceUnit distanceUnit) {
        return currentFieldPosition.getY(distanceUnit);
    }

    /**
     * Return normalized heading of current field position. Best for performing higher level
     * calculations and control.
     */
    public double getHeading() {
        return currentFieldPosition.getHeading(AngleUnit.RADIANS) % (2.0 * Math.PI); // normalize to [0, 2pi);
    }

    /**
     * Return heading of current field position in specified units as FTC "rotational convention"
     * angle, i.e., (-180, 180] degrees or (-Pi, Pi] radians from +X axis - positive
     * counter-clockwise. Best for display in telemetry.
     */
    public double getHeading(AngleUnit angleUnit) {
        if(angleUnit == AngleUnit.DEGREES) {
            double theta = currentFieldPosition.getHeading(AngleUnit.DEGREES);
            if (theta > 180) {
                return -360 - theta;
            } else {
                return theta;
            }
        }
        else {
            double theta = currentFieldPosition.getHeading(AngleUnit.RADIANS);
            if (theta > Math.PI) {
                return -2 * Math.PI - theta;
            } else {
                return theta;
            }
        }
    }

    /* ----- Mid-level motion methods for autonomous motion ----- */

    /**
     * Drive forward (reverse) while maintaining current heading and limiting sideways drift.
     * This method should be called from a LinerOpMode and implements its own loop to cover the
     * robot's motion to the specified position.
     * @param distance Distance (MM) to move: + is forward, - is reverse
     * @param speed Speed factor to apply (should use defined constants)
     */
    public void drive(double distance, double speed) {

        // Proportional controllers for x, y, and yaw
        PIDController xController = new PIDController(distance, PID_CONTROLLER_X_DEADBAND, PID_CONTROLLER_X_KP);
        PIDController yController = new PIDController(0.0, PID_CONTROLLER_Y_DEADBAND, PID_CONTROLLER_Y_KP);
        PIDController yawController = new PIDController(0.0, PID_CONTROLLER_YAW_DEADBAND, PID_CONTROLLER_YAW_KP);

        // reset the odometry counters to zero
        resetOdometryCounters();

        // Loop until the robot has reached the desired position
        while (Math.abs(xOdometryCounter - distance) > MOVE_POSITION_TOLERANCE && ((LinearOpMode) myOpMode).opModeIsActive()) {

            // Update the odometry counters
            updateOdometry();

            // Calculate the control output for each of the three controllers
            double xPower = clip(xController.calculate(xOdometryCounter), -1.0, 1.0);
            double yPower = clip(yController.calculate(yOdometryCounter), -1.0, 1.0);
            double yawPower = clip(yawController.calculate(headingOdometryCounter), -1.0, 1.0);

            // Move the robot based on the calculated powers
            move(xPower, yPower, yawPower, speed);
        }

        // stop the robot
        stop();
    }

    /**
     * Strafe left (right) while maintaining current heading and limiting forward/backward drift.
     * This method should be called from a LinerOpMode and implements its own loop to cover the
     * robot's motion to the specified position.
     * @param distance Distance (MM) to move: + is left, - is right
     * @param speed Speed factor to apply (should use defined constants)
     */
    public void strafe(double distance, double speed) {

        // Proportional controllers for x, y, and yaw
        PIDController xController = new PIDController(0.0, PID_CONTROLLER_X_DEADBAND, PID_CONTROLLER_X_KP);
        PIDController yController = new PIDController(distance, PID_CONTROLLER_Y_DEADBAND, PID_CONTROLLER_Y_KP);
        PIDController yawController = new PIDController(0.0, PID_CONTROLLER_YAW_DEADBAND, PID_CONTROLLER_YAW_KP);

        // reset the odometry counters to zero
        resetOdometryCounters();

        // Loop until the robot has reached the desired position
        while (Math.abs(yOdometryCounter - distance) > MOVE_POSITION_TOLERANCE && ((LinearOpMode) myOpMode).opModeIsActive()) {

            // Update the odometry counters
            updateOdometry();

            // Calculate the control output for each of the three controllers
            double xPower = clip(xController.calculate(xOdometryCounter), -1.0, 1.0);
            double yPower = clip(yController.calculate(yOdometryCounter), -1.0, 1.0);
            double yawPower = clip(yawController.calculate(headingOdometryCounter), -1.0, 1.0);

            // Move the robot based on the calculated powers
            move(xPower, yPower, yawPower, speed);
        }

        // stop the robot
        stop();
    }

    /**
     * Turn a relative angle while maintaining current position.
     * This method should be called from a LinerOpMode and implements its own loop to cover the
     * robot's motion to the specified position.
     * @param angle Angle to rotate in Radians: + is counter-clockwise, - is clockwise
     * @param speed Speed factor to apply (should use defined constants)
     */
    public void turn(double angle, double speed) {

        // Proportional controllers for x, y, and yaw
        PIDController xController = new PIDController(0.0, PID_CONTROLLER_X_DEADBAND, PID_CONTROLLER_X_KP);
        PIDController yController = new PIDController(0.0, PID_CONTROLLER_Y_DEADBAND, PID_CONTROLLER_Y_KP);
        PIDController yawController = new PIDController(angle, PID_CONTROLLER_YAW_DEADBAND, PID_CONTROLLER_YAW_KP);

        // reset the odometry counters to zero
        resetOdometryCounters();

        // Loop until the robot has reached the desired position
        while (Math.abs(headingOdometryCounter - angle) > ROTATE_HEADING_TOLERANCE && ((LinearOpMode) myOpMode).opModeIsActive()) {

            // Update the odometry counters
            updateOdometry();

            // Calculate the control output for each of the three controllers
            double xPower = clip(xController.calculate(xOdometryCounter), -1.0, 1.0);
            double yPower = clip(yController.calculate(yOdometryCounter), -1.0, 1.0);
            double yawPower = clip(yawController.calculate(headingOdometryCounter), -1.0, 1.0);

            // Move the robot based on the calculated powers
            move(xPower, yPower, yawPower, speed);
        }

        // stop the robot
        stop();
    }

    /* ----- High-level movement methods for autonomous motion ----- */

    /**
     * Move robot to specified field coordinate position (X, Y) in MM units
     * This method should be called from a LinerOpMode and implements its own
     * loop to cover the robots motion to the specified position.
     */
    public void moveToPosition(double x, double y, double speed) {

    }

    /**
     * Rotate robot to specified heading in field coordinate system
     * This method should be called from a LinerOpMode and implements its own
     * loop to cover the robots motion to the specified heading.
     */
    public void rotateTo(double heading, double speed) {

    }

}

/**
 * PID Controller class for computing motor power during autonomous motion
 * NOTE: This can be made just a proportional controller ("P Controller") by only passing a
 * proportional (Kp) gain.
 */
class PIDController {

    // target position
    private double target;

    // deadband range for returning zero power
    private double deadband;

    // gains
    private double Kp;
    private double Ki;
    private double Kd;

    // tracking values
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private int lastTime = 0;

    // Constructor to set the PID controller parameters with all (PID) gain values
    public PIDController(double target, double deadband, double Kp, double Ki,double Kd) {
        this.target = target;
        this.deadband = deadband;
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    // Constructor to set the controller parameters with just proportional gain. This makes the
    // controller a P controller.
    public PIDController(double target, double deadband, double Kp) {
        this.target = target;
        this.deadband = deadband;
        this.Kp = Kp;
        this.Ki = 0.0;
        this.Kd = 0.0;
    }

    // Method to calculate the control output based on the current position
    public double calculate(double currentFieldPosition) {

        // Calculate the error
        double error = target - currentFieldPosition;

        // Get elapsed time (secs) since last calculation
        int currentTime = (int) System.currentTimeMillis() / 1000;
        int deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        // Update the integral sum
        integralSum += error * deltaTime;

        // Calculate the derivative term
        double derivative = (error - lastError) / deltaTime; // rate of change of the error

        // update the last error value
        lastError = error;

        // Check if the error is within the deadband range
        if (Math.abs(error) < deadband) {
            return 0.0;
        } else {
            // Calculate the control output
            return Kp * error + Ki * integralSum + Kd * derivative;
        }
    }
}