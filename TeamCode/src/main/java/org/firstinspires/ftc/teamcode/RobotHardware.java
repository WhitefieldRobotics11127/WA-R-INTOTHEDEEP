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
 * examples, will be added as needed.
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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Hardware abstraction class for WA Robotics INTO THE DEEP competition robot
 */
public class RobotHardware {

    /* ----- Public constants (so they can be used the calling OpMode) ----- */
    // Allow drivetrain to operate in different at different, selectable "speeds"
    /** Normal speed for movement commands. */
    public static final double MOTOR_SPEED_FACTOR_NORMAL = 0.65;
    /** "Sprint" speed for movement commands. */
    public static final double MOTOR_SPEED_FACTOR_DAVIS = 1.0;
    /** Slower speed for movement commands to allow for precise odometry tracking. */
    public static final double MOTOR_SPEED_FACTOR_PRECISE = 0.35;
    /** Separate speed for Autonomous movement commands to use. */
    public static final double MOTOR_SPEED_FACTOR_AUTONOMOUS = 0.5;
    // Allowable limits for arm rotation
    // NOTE: These are [0, 1) within voltage rage of potentiometer
    /** 
     * Minimum safe rotational position for arm.
     * NOTE: Fully upright arm is the "minimum" position (within 0.0 to 1.0 range) in order to 
     * directly track with position sensor (potentiometer) values. 
     */
    public static final double ARM_ROTATION_MIN = 0.1;
    /**
     * Maximum safe rotational position for arm.
     * NOTE: Fully rotated down is the "maximum" position (within 0.0 to 1.0 range) in order to 
     * directly track with position sensor (potentiometer) values. 
     */
    public static final double ARM_ROTATION_MAX = 0.39;

    // Encoder limit for extension of arm.
    /** Encoder position for fully extended viper slide (arm) assuming fully retracted is 0. */
    public static final int ARM_EXTENSION_LIMIT = 2938;

    // Min and max limits for encoder values for extension motor
    // NOTE: These values are not static or final because they may be swapped by the reset functions
    // depending on whether it is an extended reset or a retracted reset. The ARM_EXTENSION_MIN
    // should initially be set to 0 and the ARM_EXTENSION_MAX should be set to ARM_EXTENSION_LIMIT
    // based on a fully retracted position of the viper slide when the hardware is initialized.
    /**
     * Maximum safe value for arm extension position.
     * NOTE: This may change from ARM_EXTENSION_LIMIT to 0 when arm extension encoder is reset 
     * depending on the position of the arm (fully extended or fully retracted) at reset.
     */    
    public int ARM_EXTENSION_MAX = ARM_EXTENSION_LIMIT;
    /**
     * Minimum safe value for arm extension position.
     * NOTE: This may change from 0 to -ARM_EXTENSION_LIMIT when arm extension encoder is reset 
     * depending on the position of the arm (fully extended or fully retracted) at reset.
     */
    public int ARM_EXTENSION_MIN = 0;

    // Servo positions for claw
    // NOTE: these are [0, 1) within the min and max range set for the servo
    /** Servo position for open claw. */
    public static final double CLAW_SERVO_OPEN = 0.6;
    /** Servo position for closed claw. */
    public static final double CLAW_SERVO_CLOSE = 0.08;
    /** Servo position for widest opening of claw. */
    public static final double CLAW_SERVO_OPEN_WIDE = 0.75;
    /** Servo position for tightly gripping claw. */
    public static final double CLAW_SERVO_CLOSE_GRIP = 0.0;

    /* ----- Member variables (private so hidden from the calling OpMode) ----- */

    /*
     * Parameter values for drivetrain motors.
     * Even though all robots will likely use four-motor Mecanum (Omni) drivetrains, these constants
     * may have to be modified each year based on the robots mechanical configuration, weight, and
     * performance and whether the drive motors use encoders or not.
     */
    // Correction factors for individual motors to account for mechanical differences
    // NOTE: If the robot is not driving straight, adjust these values to correct the issue.
    // NOTE: these values may not be needed if all motors are using encoders and their run modes
    // are set to RUN_USING_ENCODER
    static final double OMNI_CORRECTION_LEFT_FRONT = 1.0; // Correction factor for left front motor
    static final double OMNI_CORRECTION_RIGHT_FRONT = 1.0; // Correction factor for right front motor
    static final double OMNI_CORRECTION_LEFT_BACK = 1.0; // Correction factor for left back motor
    static final double OMNI_CORRECTION_RIGHT_BACK = 1.0; // Correction factor for right back motor

    /*
     * Parameter values for odometry calculations.
     * These are used in the calculation of the current position (relative movement and field
     * coordinates). The values are initially set from physical measurements of the robot but should
     * be tweaked for accuracy from testing (e.g., spin test and/or strafe/curve testing).
     */
    static final int DEADWHEEL_LEFT_DIRECTION = 1; // Allows for adjustment of + direction of left encoder - should be installed front to back
    static final int DEADWHEEL_RIGHT_DIRECTION = -1; // Allows for adjustment of + direction of right encoder - should be installed front to back
    static final int DEADWHEEL_AUX_DIRECTION = -1; // Allows for adjustment of + direction of aux encoder - should be installed left to right
    // The following values were calibrated for the unladen (no arm/claw assembly) robot on 10/29/2024
    static final double DEADWHEEL_MM_PER_TICK = 0.07512; // MM per encoder tick (initially calculated 48MM diameter wheel @ 2000 ticks per revolution)
    static final double DEADWHEEL_FORWARD_OFFSET = -106.0; //forward offset (length B) of aux deadwheel from robot center of rotation in MM (negative if behind)
    static final double DEADWHEEL_TRACKWIDTH = 308.4; // distance (length L) between left and right deadwheels in MM

    /*
     * Constants for autonomous motion routines.
     * These may require a lot of tweaking.
     */
    // Tolerance values for closed-loop controllers for use in translate and rotate commands
    static final double MOVE_POSITION_TOLERANCE = 12.5; // Tolerance for position in MM (~ 1/2 inch)
    static final double ROTATE_HEADING_TOLERANCE = 0.0628; // Tolerance for heading in radians (~3.6 degrees)
    static final double PID_CONTROLLER_X_DEADBAND = 5.0; // Deadband range for X power calculation. Should be less than MOVE_POSITION_TOLERANCE
    static final double PID_CONTROLLER_Y_DEADBAND = 5.0; // Deadband range for Y power calculation. Should be less than MOVE_POSITION_TOLERANCE
    static final double PID_CONTROLLER_YAW_DEADBAND = 0.03; // Deadband range for Yaw power calculation. Should be less than ROTATE_HEADING_TOLERANCE

    // PID gain values for each of the three closed-loop controllers (X, Y, and heading). These need
    // to be calibrated:
    //  - For proportional (Kp) - start with reasonable distance (error) (in mm) where the robot should start
    // to slow down while approaching the destination and take the inverse. Then adjust up until the robot
    // regularly oscillates around the target position.
    // - Once Kp is set, increase the Kd value from zero until the end behavior stabilizes.
    // - We will likely not use Ki values.
    static final double PID_CONTROLLER_X_KP = 0.0050; // Proportional gain for axial (forward) position error - start slowing down at 200 mm (~ 8 in.)
    static final double PID_CONTROLLER_X_KD = 0.0; // Derivative gain for axial (forward) position error
    static final double PID_CONTROLLER_X_KI = 0.0; // Integral gain for axial (forward) position error
    static final double PID_CONTROLLER_Y_KP = 0.0067; // Proportional gain for lateral (strafe) position error - start slowing down at 150 mm (~ 6 in.)
    static final double PID_CONTROLLER_Y_KD = 0.0; // Derivative gain for lateral (strafe) position error
    static final double PID_CONTROLLER_Y_KI = 0.0; // Integral gain for lateral (strafe) position error
    static final double PID_CONTROLLER_YAW_KP = 1.637; // Proportional gain for yaw (turning) error - start slowing down at 0.6109 radians (~ 35 degrees)
    static final double PID_CONTROLLER_YAW_KD = 0.0; // Derivative gain for yaw (turning) error
    static final double PID_CONTROLLER_YAW_KI = 0.0; // Integral gain for yaw (turning) error

    /*
     * Parameter values for arm (Viper-slide) and claw.
     */
    // Limit the power to the rotation motor to prevent damage to the arm. This needs to be calibrated.
    static final double ARM_ROTATION_POWER_LIMIT_FACTOR = 0.6; // Factor to limit power to arm rotation motor

    // Tolerances and proportional gain values for arm rotation position controller. These need to be calibrated.
    static final double ARM_ROTATION_DEADBAND = 0.01; // Deadband range for arm rotation position
    static final double ARM_ROTATION_TOLERANCE = 0.025; // Tolerance for arm rotation position
    static final double ARM_ROTATION_KP = 10.0; // Proportional gain for arm rotation position error

    // Maximum voltage from arm rotation potentiometer
    // NOTE: this is set in init() from the getMaxVoltage() method on the potentiometer and
    // utilized to calculate an arm rotation position in the [0, 1)
    // range.
    double ARM_ROTATION_MAX_VOLTAGE;
    

    // Limit the power to the extension motor to prevent damage to the arm. This needs to be calibrated.
    static final double ARM_EXTENSION_POWER_LIMIT_FACTOR = 0.7; // Factor to limit power to arm extension motor

    // Tolerances and proportional gain values for arm extension position controller. These need to be calibrated.
    static final int ARM_EXTENSION_DEADBAND = 25; // Deadband range for arm extension position in ticks (1/4 turn)
    static final double ARM_EXTENSION_KP = 0.00333; // Proportional gain for arm extension position error

    // Servo position limits for claw
    static final double CLAW_SERVO_RANGE_MIN = 0.0;
    static final double CLAW_SERVO_RANGE_MAX = 0.6;

    /*
     * Hardware objects for current robot hardware.
     * Any functionality or properties of any of these objects needed by OpModes will need to be
     * exposed through methods added to this class (thus the "abstraction" layer).
     */
    // NOTE: We should use the DCMotorEx class for all motors connected to a REV Control Hub or
    // REV Expansion whether or not  we are using RUN_USING_ENCODERS or other extended functionality
    // because the built-in REV motor controllers support all the functionality of the DCMotorEx
    // class.
    private DcMotorEx leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;  //  Motors for Mecanum drive
    private DcMotorEx rightDeadwheelEncoder, leftDeadwheelEncoder, auxDeadwheelEncoder; // Encoders (deadwheels) for odometry

    private DcMotorEx armRotationMotor, armExtensionMotor; // Motors for Viper-Slide arm extension and rotation
    private AnalogInput armRotationPositionSensor; // Potentiometer for arm rotation position
    private Servo clawServo; // Servo for claw open/close

    //private IMU imu; // IMU built into Rev Control Hub

    /*
     * Variables for tracking robot state     
     */
    // last read odometry deadwheel encoder positions - used to calculate encoder deltas since last
    // call to updateOdometry()
    // ***** NOTE: These are made public temporarily for initial testing/tuning purposes *****
    public int lastRightEncoderPosition, lastLeftEncoderPosition, lastAuxEncoderPosition;

    // translated x, y, and heading odometry counters in mm since last reset
    // NOTE: these are updated by the updateOdometry() method and used for simple movement commands
    // (forward, strafe, turn) as well as calculating field position.
    private double xOdometryCounter, yOdometryCounter, headingOdometryCounter;
    
    // Current robot position (x,y, heading) in field coordinate system
    // NOTE: this is updated by the updateOdometry() method and used for translation and/or rotation
    // to field coordinates
    private Pose2D currentFieldPosition = new Pose2D(DistanceUnit.MM, 0,0,AngleUnit.RADIANS,0);

    // keep a reference to the calling opmode so that we have access to hardwareMap and other
    // properties and statuses from the running opmode.
    private final OpMode myOpMode;

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
        // motor controllers no." Also, there are questions of whether a short sleep is needed
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
        // call, a myOpMode.idle() call (only for Linear OpModes), etc.?
        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // may not be needed
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Define odometry encoder hardware instance variables
        rightDeadwheelEncoder = myOpMode.hardwareMap.get(DcMotorEx.class, "encoder_right");
        leftDeadwheelEncoder = myOpMode.hardwareMap.get(DcMotorEx.class, "encoder_left");
        auxDeadwheelEncoder = myOpMode.hardwareMap.get(DcMotorEx.class, "encoder_aux");

        // Reset the encoder values
        rightDeadwheelEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftDeadwheelEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        auxDeadwheelEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Define arm and claw hardware instance variables
        // NOTE: ****** The rotation motor uses the same motor port as the auxiliary deadwheel encoder
        armRotationMotor = auxDeadwheelEncoder; // Use the same hardware mapping as the auxiliary odometry encoder
        armExtensionMotor = myOpMode.hardwareMap.get(DcMotorEx.class, "arm_extension");
        armRotationPositionSensor = myOpMode.hardwareMap.get(AnalogInput.class, "arm_rot_pos");
        clawServo = myOpMode.hardwareMap.get(Servo.class, "claw_servo");

        // Initialize settings for viper-slide arm
        armExtensionMotor.setDirection(DcMotorEx.Direction.REVERSE);
        armExtensionMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Reset the encoder count to zero and make sure the motor is stopped
        // NOTE: the viper slide should be fully retracted before "Start" is pressed
        armExtensionMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armExtensionMotor.setPower(0.0);

        // Set the run mode to RUN_USING_ENCODER
        armExtensionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize settings for arm motor
        // NOTE: Run the motor in forward direction so that power to the motor is positive when
        // tilting down. This allows the direction of rotation to track with the direction of the
        // rotation position sensor (potentiometer).
        // NOTE: ***** Setting these values wrong can cause the arm and arm rotation hardware
        // to be damaged because the arm can be over-rotated and the gearbox on the motor makes it
        // very strong. *****
        armRotationMotor.setDirection(DcMotorEx.Direction.FORWARD);
        armRotationMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armRotationMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Get the maximum voltage from the arm rotation potentiometer for calculating arm rotation
        // position in the [0, 1) range.
        ARM_ROTATION_MAX_VOLTAGE = armRotationPositionSensor.getMaxVoltage();

        // Initialize settings for claw servo
        // Set a limited range for claw servo to prevent damage to the claw
        clawServo.scaleRange(CLAW_SERVO_RANGE_MIN, CLAW_SERVO_RANGE_MAX);

        // Define IMU hardware instance variable
        //imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        // Set the IMU orientation on the robot
        //imu.initialize(
        //        new IMU.Parameters(
        //                new RevHubOrientationOnRobot(
        //                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
        //                    RevHubOrientationOnRobot.UsbFacingDirection.UP
        //                )
        //        )
        //);
    }

    /* ----- Low level motion methods for four-motor Mecanum drive train ----- */

    /**
     * Set Mecanum drivetrain motor powers directly.
     * Applies any defined correction values.
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
     * Drive robot according to robot-oriented axes of motion.
     * This method can be used by teleop OpModes directly to drive the robot, since the human on
     * the gamepad will be viewing and controlling the robot on the field with subtle adjustments
     * (thus "closed-loop" controller). It is also called by the higher-level motion routines in
     * the RobotHardware class for autonomous driving.
     * @param x "power" (relative speed) for axial movement (+ is forward)
     * @param y power for lateral movement (strafe) (+ is left)
     * @param yaw power for rotation (+ is counter-clockwise)
     * @param speed factor to scale the power values (0.0 to 1.0). Use MOTOR_SPEED_FACTOR_NORMAL, MOTOR_SPEED_FACTOR_DAVIS, or MOTOR_SPEED_FACTOR_PRECISE
     */
    public void move(double x, double y, double yaw, double speed) {

        // Calculate the powers for the four motors attached to the Mecanum wheels based on the
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
     * Stop robot motion.
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
     * sub-loops in translation or rotation routines.
     */
    public void updateOdometry() {

        /*
         * NOTE: This code is adapted from the discussion of odometry in Game Manual 0 found here:
         * https://gm0.org/en/latest/docs/software/concepts/odometry.html. The code currently
         * implements linear approximations of deltas, i.e., assuming that the individual x and y
         * movements between updates occurred in straight lines. This may be fine for the mid-level
         * autonomous movement functions (forward, strafe, turn), but may build up errors over time
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
        lastRightEncoderPosition = rightDeadwheelEncoder.getCurrentPosition() * DEADWHEEL_RIGHT_DIRECTION;
        lastLeftEncoderPosition = leftDeadwheelEncoder.getCurrentPosition() * DEADWHEEL_LEFT_DIRECTION;
        lastAuxEncoderPosition = auxDeadwheelEncoder.getCurrentPosition() * DEADWHEEL_AUX_DIRECTION;

        // calculate x, y, n1                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       and theta (heading) deltas (robot perspective) since last measurement
        int dl = lastLeftEncoderPosition  - oldLeftCounter;
        int dr= lastRightEncoderPosition - oldRightCounter;
        int da = lastAuxEncoderPosition - oldAuxOdometryCounter;
        double dtheta = DEADWHEEL_MM_PER_TICK * (dr-dl) / DEADWHEEL_TRACKWIDTH; // this approximation seems like it would build up a lot of error
        //double dtheta = Math.acos(1 - Math.pow(DEADWHEEL_MM_PER_TICK * (dr -dl), 2) / (2 * Math.pow(DEADWHEEL_TRACKWIDTH, 2))); // this seemed to be more technically accurate but wasn't working?
        double dx = DEADWHEEL_MM_PER_TICK * (dl+dr) / 2.0;
        double dy = DEADWHEEL_MM_PER_TICK * da - DEADWHEEL_FORWARD_OFFSET * dtheta;

        // update the x, y, and heading odometry counters
        xOdometryCounter += dx;
        yOdometryCounter += dy;
        headingOdometryCounter += dtheta;

        // update the current position in field coordinate system from the deltas
        // NOTE: disable this code for now since we are not using the field position and it may be
        // making the runtime too long. Utlimately we may want to store the field position in a
        // structure that is not as intensive to update (i.e., one where a new object instance
        // doesn't have to be created each time).
        /*
        double theta = currentFieldPosition.getHeading(AngleUnit.RADIANS) + (dtheta / 2);
        double newX = currentFieldPosition.getX(DistanceUnit.MM) + dx * Math.cos(theta) - dy * Math.sin(theta);
        double newY = currentFieldPosition.getY(DistanceUnit.MM) + dx * Math.sin(theta) + dy * Math.cos(theta);
        double newHeading = (currentFieldPosition.getHeading(AngleUnit.RADIANS) + dtheta) % (2.0 * Math.PI); // normalize to [0, 2pi)
        if(newHeading < 0) {
            newHeading += 2.0 * Math.PI;
        }
        currentFieldPosition = new Pose2D(DistanceUnit.MM, newX, newY, AngleUnit.RADIANS, newHeading);
        */
    }

    /**
     * Reset the x, y, and heading odometry counters to zero.
     * This method should be called at the beginning of simple move and rotate commands to ensure
     * that translation and rotation are relevant to robot's starting position.
     */
    public void resetOdometryCounters() {

        // reset the odometry counters to zero
        xOdometryCounter = 0.0;
        yOdometryCounter = 0.0;
        headingOdometryCounter = 0.0;
    }

    /**
     * Return axial (x) odometry counter in MM units.
     * This method is primarily for retrieval of the odometry counters by the OpMode for display
     * in telemetry during testing.
     */
    public double getOdometryX() {
        return xOdometryCounter;
    }

    /**
     * Return lateral (y) odometry counter in MM units.
     * This method is primarily for retrieval of the odometry counters by the OpMode for display
     * in telemetry during testing.
     */
    public double getOdometryY() {
        return yOdometryCounter;
    }

    /**
     * Return heading odometry counter in radians.
     * This method is primarily for retrieval of the odometry counters by the OpMode for display
     * in telemetry during testing.
     */
    public double getOdometryHeading() {
        return headingOdometryCounter;
    }

    /**
     * Set the robot's position in the field coordinate system in MM and radians.
     * This method should be called to initialize the robot's initial position from known starting
     * position (e.g., from the field setup or from a known position on the field). This method may
     * also be called when updating the robot's position from vision processing or other sensors.
     * @param x x-coordinate of center of robot in field coordinates (MM)
     * @param y y-coordinate of center of robot in field coordinates (MM)
     * @param heading current angle of robot relative to positive x-axis in field coordinates (rad)
     */
    public void setFieldPosition(double x, double y, double heading) {
        currentFieldPosition = new Pose2D(DistanceUnit.MM, x, y, AngleUnit.RADIANS, heading);
    }

    /**
     * Set the robot's position in the field coordinate system in specified distance and angle
     * units.
     * @param x x-coordinate of center of robot in field coordinates
     * @param y y-coordinate of center of robot in field coordinates
     * @param dUnit distance unit for x and y coordinates
     * @param heading current angle of robot relative to positive x-axis in field coordinates
     * @param aUnit angle unit for heading
     */
    public void setFieldPosition(double x, double y, DistanceUnit dUnit, double heading, AngleUnit aUnit) {
        currentFieldPosition = new Pose2D(dUnit, x, y, aUnit, heading);
    }

    /**
     * Return current field position as a Pose2D object.
     */
    public Pose2D getCurrentFieldPosition() {
        return currentFieldPosition;
    }

    /**
     * Return X coordinate of current field position in MM units.
     */
    public double getFieldPosX() {
        return currentFieldPosition.getX(DistanceUnit.MM);
    }

    /**
     * Return X coordinate of current field position in specified units.
     */
    public double getFieldPosX(DistanceUnit distanceUnit) {
        return currentFieldPosition.getX(distanceUnit);
    }

    /**
     * Return Y coordinate of current field position in MM units.
     */
    public double getFieldPosY() {
        return currentFieldPosition.getY(DistanceUnit.MM);
    }

    /**
     * Return Y coordinate of current field position in specified units.
     */
    public double getFieldPosY(DistanceUnit distanceUnit) {
        return currentFieldPosition.getY(distanceUnit);
    }

    /**
     * Return heading of current field position. Best for performing higher level
     * calculations and control.
     */
    public double getFieldHeading() {
        return currentFieldPosition.getHeading(AngleUnit.RADIANS);
    }

    /**
     * Return heading of current field position in specified units.
     * This method converts the angle to FTC "rotational convention" angle, i.e.,
     * (-180, 180] degrees or (-Pi, Pi] radians from +X axis - positive counter-clockwise. Best for
     * display in telemetry.
     * @param angleUnit the angle unit (AngleUnit.DEGREES or AngleUnit.RADIANS) in which to return the heading
     */
    public double getFieldHeading(AngleUnit angleUnit) {
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
     * This method should only be called from a LinerOpMode and implements its own loop to cover the
     * robot's motion to the specified position.
     * @param distance Distance (MM) to move: + is forward, - is reverse
     * @param speed Speed factor to apply (should use defined constants)
     */
    public void forward(double distance, double speed) {

        // Proportional controllers for x, y, and yaw
        //PIDController xController = new PIDController(distance, PID_CONTROLLER_X_DEADBAND, PID_CONTROLLER_X_KP, PID_CONTROLLER_X_KI, PID_CONTROLLER_X_KD);
        //PIDController yController = new PIDController(0.0, PID_CONTROLLER_Y_DEADBAND, PID_CONTROLLER_Y_KP, PID_CONTROLLER_Y_KI, PID_CONTROLLER_Y_KD);
        //PIDController yawController = new PIDController(0.0, PID_CONTROLLER_YAW_DEADBAND, PID_CONTROLLER_YAW_KP, PID_CONTROLLER_YAW_KI, PID_CONTROLLER_YAW_KD);

        // Flag to determine if called from a Liner OpMode
        boolean isLinearOpMode = myOpMode instanceof LinearOpMode;

        // reset the odometry counters to zero
        resetOdometryCounters();

        // Loop until the robot has reached the desired position
        while (Math.abs(distance - xOdometryCounter) > MOVE_POSITION_TOLERANCE && (!isLinearOpMode || ((LinearOpMode) myOpMode).opModeIsActive())) {

            // Calculate the control output for each of the three controllers
            //double xPower = clip(xController.calculate(xOdometryCounter), -1.0, 1.0);
            double xPower = clip(PID_CONTROLLER_X_KP * (distance - xOdometryCounter), -1.0, 1.0);
            //double yPower = clip(yController.calculate(yOdometryCounter), -1.0, 1.0);
            double yPower = 0;
            //double yawPower = clip(yawController.calculate(headingOdometryCounter), -1.0, 1.0);
            double yawPower = 0;

            // Move the robot based on the calculated powers
            move(xPower, yPower, yawPower, speed);

            // If we are in a linear opmode, sleep for a short time to allow the robot to move
            // and/or for the encoder wheels to update their position.
            // NOTE: Need to look into more what's needed/efficient here: sleep(), idle(), or nothing
            // at all.
            if (isLinearOpMode) {
                //((LinearOpMode) myOpMode).sleep(50);
                ((LinearOpMode) myOpMode).idle();
            }

            // Update the odometry counters
            updateOdometry();
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
        //PIDController xController = new PIDController(0.0, PID_CONTROLLER_X_DEADBAND, PID_CONTROLLER_X_KP, PID_CONTROLLER_X_KI, PID_CONTROLLER_X_KD);
        //PIDController yController = new PIDController(distance, PID_CONTROLLER_Y_DEADBAND, PID_CONTROLLER_Y_KP, PID_CONTROLLER_Y_KI, PID_CONTROLLER_Y_KD);
        //PIDController yawController = new PIDController(0.0, PID_CONTROLLER_YAW_DEADBAND, PID_CONTROLLER_YAW_KP, PID_CONTROLLER_YAW_KI, PID_CONTROLLER_YAW_KD);

        // Flag to determine if called from a Liner OpMode
        boolean isLinearOpMode = myOpMode instanceof LinearOpMode;

        // reset the odometry counters to zero
        resetOdometryCounters();

        // Loop until the robot has reached the desired position
        while (Math.abs(distance - yOdometryCounter) > MOVE_POSITION_TOLERANCE && (!isLinearOpMode || ((LinearOpMode) myOpMode).opModeIsActive())) {

            // Calculate the control output for each of the three controllers
            //double xPower = clip(xController.calculate(xOdometryCounter), -1.0, 1.0);
            double xPower = 0;
            //double yPower = clip(yController.calculate(yOdometryCounter), -1.0, 1.0);
            double yPower = clip(PID_CONTROLLER_Y_KP * (distance - yOdometryCounter), -1.0, 1.0);
            //double yawPower = clip(yawController.calculate(headingOdometryCounter), -1.0, 1.0);
            double yawPower = 0;

            // Move the robot based on the calculated powers
            move(xPower, yPower, yawPower, speed);

            // If we are in a linear opmode, sleep for a short time to allow the robot to move
            // and/or for the encoder wheels to update their position.
            // NOTE: Need to look into more what's needed/efficient here: sleep(), idle(), or nothing
            // at all.
            if (isLinearOpMode) {
                //((LinearOpMode) myOpMode).sleep(50);
                ((LinearOpMode) myOpMode).idle();
            }

            // Update the odometry counters
            updateOdometry();
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
        //PIDController xController = new PIDController(0.0, PID_CONTROLLER_X_DEADBAND, PID_CONTROLLER_X_KP, PID_CONTROLLER_X_KI, PID_CONTROLLER_X_KD);
        //PIDController yController = new PIDController(0.0, PID_CONTROLLER_Y_DEADBAND, PID_CONTROLLER_Y_KP, PID_CONTROLLER_Y_KI, PID_CONTROLLER_Y_KD);
        //PIDController yawController = new PIDController(angle, PID_CONTROLLER_YAW_DEADBAND, PID_CONTROLLER_YAW_KP, PID_CONTROLLER_YAW_KI, PID_CONTROLLER_YAW_KD);

        // Flag to determine if called from a Liner OpMode
        boolean isLinearOpMode = myOpMode instanceof LinearOpMode;

        // reset the odometry counters to zero
        resetOdometryCounters();

        // Loop until the robot has reached the desired position
        while (Math.abs(angle - headingOdometryCounter) > ROTATE_HEADING_TOLERANCE && (!isLinearOpMode || ((LinearOpMode) myOpMode).opModeIsActive())) {

            // Calculate the control output for each of the three controllers
            //double xPower = clip(xController.calculate(xOdometryCounter), -1.0, 1.0);
            double xPower = 0;
            //double yPower = clip(yController.calculate(yOdometryCounter), -1.0, 1.0);
            double yPower = 0;
            //double yawPower = clip(yawController.calculate(headingOdometryCounter), -1.0, 1.0);
            double yawPower = clip(PID_CONTROLLER_YAW_KP * (angle - headingOdometryCounter), -1.0, 1.0);

            // Move the robot based on the calculated powers
            // NOTE: We reduced the yaw power by 40% to make the robot turn more slowly and accurately
            move(xPower, yPower, yawPower, speed * 0.7);

            // If we are in a linear opmode, sleep for a short time to allow the robot to move
            // and/or for the encoder wheels to update their position.
            // NOTE: Need to look into more what's needed/efficient here: sleep(), idle(), or nothing
            // at all.
            if (isLinearOpMode) {
                //((LinearOpMode) myOpMode).sleep(50);
                ((LinearOpMode) myOpMode).idle();
            }

            // Update the odometry counters
            updateOdometry();
        }

        // stop the robot
        stop();
    }

    /* ----- High-level movement methods for autonomous motion ----- */

    /**
     * Move robot to specified field coordinate position (X, Y) and heading in MM and radians.
     * This method should only be called from a LinerOpMode and implements its own
     * loop to cover the robots motion to the specified position.
     * @param x x-coordinate of center of robot in field coordinates (MM)
     * @param y y-coordinate of center of robot in field coordinates (MM)
     * @param heading current angle of robot relative to positive x-axis in field coordinates (rad)
     * @param speed Speed factor to apply (should use defined constants)
     */
    public void moveToPosition(double x, double y, double heading, double speed) {

    }

    /* ----- Arm and claw control methods ----- */

    /**
     * Power the motor to raise (-) or lower (+) the arm.
     * Applies the specified power to the arm rotation motor while monitoring the
     * potentiometer position and keeping the arm within limits.
     * @param power power for the arm rotation motor (-1.0 to 1.0)
     */
    public void rotateArm(double power) {

        // Apply a proportional gain to the power as the position approaches the limits.
        // NOTE: We can't use the PID controller class here since this function is called from
        // teleop OpModes inside the loop() method and the PID controller can't be (easily)
        // persisted across calls
        double currentPosition = armRotationPositionSensor.getVoltage() / ARM_ROTATION_MAX_VOLTAGE;
        double error;

        if (power < 0.0) {
            error = currentPosition - ARM_ROTATION_MIN;
        }
        else {
            error = ARM_ROTATION_MAX - currentPosition;
        }

        //armRotationMotor.setPower(power);
        if (Math.abs(error) > ARM_ROTATION_DEADBAND) {
            armRotationMotor.setPower(power * clip(ARM_ROTATION_KP * error, -1.0, 1.0) * ARM_ROTATION_POWER_LIMIT_FACTOR);
        }
        else {
            armRotationMotor.setPower(0.0);
        }
    }

    /**
     * Stop the arm from rotation.
     */
    public void stopArmRotation() {
         armRotationMotor.setPower(0.0);
    }

    /**
     * Retrieve the current position value for the arm rotation (0.0 to 1.0).
     */
    public double getArmRotation() {
        // return the current position values the arm calculated as a proportion of the maximum
        // voltage from the potentiometer
        return armRotationPositionSensor.getVoltage() / ARM_ROTATION_MAX_VOLTAGE;
    }

    /**
     * Retrieve raw voltage for arm rotation potentiometer (volts).
     * This method can be used to display telemetry information during testing and calibration.
     */
    public double getArmRotationSensorVoltage() {
        // return the current potentiometer value for the arm rotation angle
        return armRotationPositionSensor.getVoltage();
    }

    /**
     * Retrieve current power setting for the arm rotation motor (-1.0 to 1.0).
     * This method can be used to display telemetry information during testing and calibration.
     */
    public double getArmRotationMotorPower() {
        return armRotationMotor.getPower();
    }

    /**
     * Rotate the arm to a specified rotation position.
     * This method should only be called from a LinerOpMode and implements its own
     * loop to cover the rotation of the arm the specified position using a PID controller with
     * the potentiometer value and specified position for determination of error.
     * @param position the desired "angle" for the arm rotation motor (ARM_ROTATION_MIN to ARM_ROTATION_MAX)
     */
    public void setArmRotation(double position) {

        // Flag to determine if called from a Liner OpMode
        boolean isLinearOpMode = myOpMode instanceof LinearOpMode;

        // Make sure it's in the allowable range
        position = clip(position, ARM_ROTATION_MIN, ARM_ROTATION_MAX);

        // Create a PID controller for the arm rotation position potentiometer
        //PIDController armRotationController = new PIDController(position, ARM_ROTATION_DEADBAND, ARM_ROTATION_KP);

        // get the initial potentiometer value for the arm rotation angle
        double armPosition = armRotationPositionSensor.getVoltage() / ARM_ROTATION_MAX_VOLTAGE;

        // Loop until the arm rotation has reached the desired position
        while (Math.abs(position - armPosition) > ARM_ROTATION_TOLERANCE && (!isLinearOpMode || ((LinearOpMode) myOpMode).opModeIsActive())) {

            // Calculate the voltage output for the arm rotation motor
            //double power = clip(armRotationController.calculate(armPosition), -1.0, 1.0);
            double power = clip(ARM_ROTATION_KP * (position - armPosition), -1.0, 1.0);

            // Rotate the arm
            armRotationMotor.setPower(power * ARM_ROTATION_POWER_LIMIT_FACTOR);

            // Give the arm some time to move
            if (isLinearOpMode) {
                ((LinearOpMode) myOpMode).sleep(50);
            }

            // Update the potentiometer value for the arm rotation angle
            armPosition = armRotationPositionSensor.getVoltage() / ARM_ROTATION_MAX_VOLTAGE;
        }

        // Stop the rotation of the arm
        armRotationMotor.setPower(0.0);
     }

    /**
     * Power the motor to extend (+) or retract (-) the slide.
     * Apply the specified power to the arm extension motor while monitoring the
     * encoder position and keeping the arm within limits.
     * @param power power for the arm extension motor (-1.0 to 1.0)
     */
    public void extendArm(double power) {

        // Apply a proportional gain to the power as the position approaches the limits.
        // NOTE: We can't use the PID controller class here since this function is called from
        // teleop OpModes inside the loop() method and the PID controller can't be (easily)
        // persisted across calls.
        // NOTE: Another way to do this may be to set the motor in RUN_TO_POSITION mode and change
        // the target position and or otherwise reset the operation when the sign of the power
        // changes or is set to zero.
        double currentPosition = armExtensionMotor.getCurrentPosition();
        double error;

        if (power < 0.0)
            error = currentPosition - ARM_EXTENSION_MIN;
        else
            error = ARM_EXTENSION_MAX - currentPosition;

        if (Math.abs(error) > ARM_EXTENSION_DEADBAND)
            armExtensionMotor.setPower(power * clip(ARM_EXTENSION_KP * error, -1.0,1.0) * ARM_EXTENSION_POWER_LIMIT_FACTOR);
        else
            armExtensionMotor.setPower(0.0);
    }

    /**
     * Use the motor controllers RUN_TO_POSITION function to extend the arm to the specified
     * position (encoder value).
     * This function is for use in a Teleop OpMode and sets the target position and applies power.
     * The calling OpMode must set the power on the motor back to zero when done (not busy).
     * @param position the desired position for the extension, between 0 and ARM_EXTENSION_LIMIT
     */
    public void extendArmToPosition(int position) {

        // make sure the position is within the allowable range
        position = clip(position, ARM_EXTENSION_MIN, ARM_EXTENSION_MAX);

        // set up the motor for run to position with the target encoder position
        armExtensionMotor.setTargetPosition(position);
        armExtensionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Power the motor to extend the arm
        armExtensionMotor.setPower(ARM_EXTENSION_POWER_LIMIT_FACTOR);
    }

    /**
     * Return isBusy state of arm extension motor.
     */
    public boolean isArmExtensionBusy() {

        return (armExtensionMotor.isBusy());
    }

    /**
     * Stop the arm extension motor from moving. Should be called after call to extendArmToPosition
     * is complete.
     */public void stopArmExtension() {

        armExtensionMotor.setPower(0.0);
        armExtensionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Retrieve the current arm extension position.
     * This method can be used to display telemetry information during testing.
     */
    public int getArmExtension() {
        // return the current encoder value for the arm extension motor
        return armExtensionMotor.getCurrentPosition();
    }

    /**
     * Extend the arm to the specified position (encoder value).
     * This method should only be called from a LinerOpMode and implements its own
     * loop to cover the extension of the arm the specified position using RUN_TO_ENCODER in the
     * arm extension motor (with built-in PID controller).
     * @param position the desired position for the extension, between 0 and ARM_EXTENSION_LIMIT
     */
    public void setArmExtension(int position) {

        // Flag to determine if called from a Liner OpMode
        boolean isLinearOpMode = myOpMode instanceof LinearOpMode;

        // set up the motor for run to position with the target encoder position
        armExtensionMotor.setTargetPosition(position);
        armExtensionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Power the motor to extend the arm
        armExtensionMotor.setPower(ARM_EXTENSION_POWER_LIMIT_FACTOR);

        // loop until the arm extension has reached the desired position
        while (armExtensionMotor.isBusy()) {

            // Wait for the motor to reach the target position
            if (isLinearOpMode)
                ((LinearOpMode) myOpMode).idle();
        }

        // stop the motor
        armExtensionMotor.setPower(0.0);

        // reset the motor settings
        armExtensionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Reset the limits for arm extension from fully extended arm.
     * This method should be called when the arm is fully extended, i.e. over limit.
     */
    public void resetArmLimitsExtended() {

        // reset the encoder for the arm extension motor
        armExtensionMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // reset the limit values accordingly
        ARM_EXTENSION_MAX = 0;
        ARM_EXTENSION_MIN = -ARM_EXTENSION_LIMIT;

        // set the mode back to RUN_USING_ENCODER
        armExtensionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Reset the limits for arm extension from fully retracted arm.
     * This method should be called when the arm is fully retracted, i.e., under limit.
     */
    public void resetArmLimitsRetracted() {

        // reset the encoder for the arm extension motor
        armExtensionMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // reset the limit values accordingly
        ARM_EXTENSION_MAX = ARM_EXTENSION_LIMIT;
        ARM_EXTENSION_MIN = 0;

        // set the mode back to RUN_USING_ENCODER
        armExtensionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Open the claw.
     * @param wide true to open the claw to the wide position, false to open to the normal open position
     */
    public void openClaw(boolean wide) {
        // Possible usage is to open the claw wide as long as the driver is holding the trigger and
        // then return it to the normal open position when the trigger is released.
        if (wide)
            clawServo.setPosition(CLAW_SERVO_OPEN_WIDE);
        else
            clawServo.setPosition(CLAW_SERVO_OPEN);
    }

    /**
     * Close the claw.
     * @param grip true to close the claw to the tight or "grip" position, false to close to the normal closed position
     */
    public void closeClaw(boolean grip) {
        // Possible usage is to close the claw tightly as long as the driver is holding the trigger
        // and then return it to the normal closed position when the trigger is released.
        if(grip)
            clawServo.setPosition(CLAW_SERVO_CLOSE_GRIP);
        else
            clawServo.setPosition(CLAW_SERVO_CLOSE);
    }

    /**
     * Retrieve the current claw servo position setting (0.0 to 1.0).
     * This method can be used to display telemetry information during testing and calibration.
     */
    public double getClawPosition() {
        return clawServo.getPosition();
    }

    /* ----- Vision processing methods ----- */

    // TODO: Add AprilTag detection functionality here, including a method to initialize the
    // VisionPortal and AprilTagProcessor objects
}

/**
 * PID Controller class for computing motor power during autonomous motion.
 * NOTE: This can be made just a proportional controller ("P Controller") by only passing a
 * proportional (Kp) gain.
 */
class PIDController {

    // target position
    private final double target;

    // deadband range for returning zero power
    private final double deadband;

    // gains
    private final double Kp;
    private final double Ki;
    private final double Kd;

    // tracking values
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private int lastTime = 0;

    // Constructor to set the PID controller parameters with all (PID) gain values
    public PIDController(double target, double deadband, double Kp, double Ki, double Kd) {
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
    public double calculate(double currentPosition) {

        // Calculate the error
        double error = target - currentPosition;

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
        if (Math.abs(error) < deadband)
            return 0.0;
        else
            // Calculate the control output
            return Kp * error + Ki * integralSum + Kd * derivative;
    }
}
