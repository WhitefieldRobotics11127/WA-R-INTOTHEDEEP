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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
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

    /* ----- Public constants (so they CAN be used the calling OpMode - just in case) ----- */

    /* Parameter values for drivetrain motors.
     * Even though all robots will likely use four-motor Mecanum (Omni) drivetrains, these constants
     * may have to be modified each year based on the robots mechanical configuration, weight, and
     * performance
     */
    
    // Allow drivetrain to operate in different at different, selectable "speeds"
    public  static final double MOTOR_SPEED_FACTOR_NORMAL = 0.65; // Normal power limit
    public static final double MOTOR_SPEED_FACTOR_SPRINT = 1.0; // Sprint power limit
    public static final double MOTOR_SPEED_FACTOR_PRECISE = 0.4; // Precise positioning power limit
    
    // Correction factors for individual motors to account for mechanical differences
    // NOTE: If the robot is not driving straight, adjust these values to correct the issue.
    // NOTE: these values may not be needed if all motors are using encoders and the run modes
    // are set to RUN_USING_ENCODER
    public static final double OMNI_CORRECTION_LEFT_FRONT = 1.0; // Correction factor for left front motor
    public static final double OMNI_CORRECTION_RIGHT_FRONT = 1.0; // Correction factor for right front motor
    public static final double OMNI_CORRECTION_LEFT_BACK = 1.0; // Correction factor for left back motor
    public static final double OMNI_CORRECTION_RIGHT_BACK = 1.0; // Correction factor for right back motor

    /* Parameter values for Odometry calulations.
     * These are used in the calculation of the current position (relative movement and field
     * coordinates). The values are initially set from physical measurements of the robot but should
     * be tweaked for accuracy from testing (e.g., spin test and/or strafe/curve testing).
     */
    public static final double DEADWHEEL_MM_PER_TICK = 0.0754; // MM per encoder tick (48MM diameter wheel @ 2000 ticks per revolution)
    public static final double DEADWHEEL_FORWARD_OFFSET = 138.55; //forward offset (length B) of aux deadwheel from robot center of rotation in MM
    public static final double DEADWHEEL_TRACKWIDTH = 332.51; // distance (length L) between left and right deadwheels in MM

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
    public static final double ROTATE_HEADING_TOLERANCE = 0.0174533; // Tolerance for heading in radians (1 degree)

    // PID gain values for each of the three closed-loop controllers (X, Y, and heading)
    public static final double PID_CONTROLLER_X_KP = 0.16; // Proportional gain for X position
    public static final double PID_CONTROLLER_X_KI = 0.0; // Integral gain for X position
    public static final double PID_CONTROLLER_X_KD = 0.0; // Derivative gain for X position
    public static final double PID_CONTROLLER_Y_KP = 0.1; // Tolerance for Y position
    public static final double PID_CONTROLLER_Y_KI = 0.0; // Integral gain for Y position
    public static final double PID_CONTROLLER_Y_KD = 0.0; // Derivative gain for Y position
    public static final double PID_CONTROLLER_HEADING_KP = 0.1; // Proportional gain for heading
    public static final double PID_CONTROLLER_HEADING_KI = 0.0; // Integral gain for heading
    public static final double PID_CONTROLLER_HEADING_KD = 0.0; // Derivative gain for heading
    
    /* ----- Member variables (private to hide from the calling opmode) ----- */

    /*
     * Hardware objects for current robot hardware.
     * Any functionality or properties of any of these objects needed by opmodes will need to be
     * exposed through methods added to this class (thus the "abstraction" layer).
     */
    private DcMotorEx leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;  //  Motors for Mecanum drive
    private DcMotor encoderRight, encoderLeft, encoderAux; // Encoders (deadwheels) for odometry
    private IMU imu; // IMU built into Rev Control Hub

    //private DcMotor armRotation, armExtension; // Motors for Viper-Slide arm extension and rotation
    //private Servo clawServo; // Servo for claw open/close

    private VisionPortal visionPortal; // Used to manage the video source.
    private AprilTagProcessor aprilTag; // Used for managing the AprilTag detection processor

    /*
     * Variables for tracking robot state     
     */
    // last read odometry deadwheel encoder positions 
    // NOTE: these are used to calculate encoder deltas since last call to updateOdometry() 
    private int lastRightEncoderPosition, lastLeftEncoderPosition, lastAuxEncoderPosition;

    // translated x, y, and heading odometry counters in mm since last reset
    // NOTE: these are updated by the updateOdometry() method and used for simple movement commands
    // (drive, strafe, rotate).
    private double xOdometryOdometryCounter, yOdometryCounter, headingOdometryCounter;
    
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

        // Set the run mode, braking, and direction for each motor, make sure robot it not moving
        // (power to zero and/or stop commands), and initialize encoders, etc.
        // NOTE: There may be order and/or timing issues here, e.g., setting the runmode to
        // RUN_USING_ENCODER may need to happen after setting the runmode to STOP_AND_RESET_ENCODER,
        // and after a short sleep by the opmode (e.g., 100ms) to allow the encoders to reset.
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // What do we need here to ensure that the encoders are reset before we set the run mode to
        // RUN_USING_ENCODER? A Thread.sleep() call, a myOpMode.wait() call, a myOpMode.idle() call
        // (only for Linear opmodes), etc.?

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // increased accuracy and balance from controls
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define odometry encoder hardware instance variables
        encoderRight = myOpMode.hardwareMap.get(DcMotor.class, "encoder_right");
        encoderLeft = myOpMode.hardwareMap.get(DcMotor.class, "encoder_left");
        encoderAux = myOpMode.hardwareMap.get(DcMotor.class, "encoder_aux");

        // Define IMU hardware instance variable
        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        // Define arm and claw hardware instance variables
        //armRotation = myOpMode.hardwareMap.get(DcMotor.class, "arm_rotation");
        //armExtensions = myOpMode.hardwareMap.get(DcMotor.class, "arm_extension");
        //clawServo = myOpMode.hardwareMap.get(Servo.class, "claw_servo");

    }

    /* ----- Low level motion methods for four-motor Mecanum drive train ----- */

    /**
    * Set Mecanum drivetrain motor powers
    */
    public void setMotorPowers(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {

        // Send powers to the wheels.
        // NOTE: since we are using encoders on all wheels, what we are really doing here is
        // specifying a percentage (0.0 to 1.0) of the maximum RPM speed of the motor. Similar to
        // calling setSpeed() but without having to know the speed ranges and values. The motor
        // controller code (DCMotor or DCMotorEx classes) can then take care of changing voltage
        // levels from the battery and variations in friction in the motors for weight distribution
        // of the robot to provide more balanced speed control to the four wheels. The actual speed
        // of the motor is managed by the DCMotor (or DCMotorEx) class utilizing a built-in PID
        // controller to attain the calculated RPM. We may want to adjust the PID controller gain
        // values via methods in the DCMotorEx class to obtain stable operation.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Drive robot according to robot-oriented axes of motion
     * This method can be used by telop opmodes directly to drive the robot, since the human on
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

    /* ----- Methods for three-wheel odometry ----- */

    /**
     * Read odometry wheel encoders and update the current odometry counters and field position
     * of the robot.
     * This method should be called at the beginning of each loop in an autonomous opMode and in any
     * subloops in translation or rotation routines.
     */
    public void updateOdometry() {

        // save current encoder values
        int oldRightCounter = lastRightEncoderPosition;
        int oldLeftCounter = lastLeftEncoderPosition;
        int oldAuxOdometryCounter = lastAuxEncoderPosition;

        // read new encoder values from odometry deadwheels
        lastRightEncoderPosition = encoderRight.getCurrentPosition(); // check sign based on installation
        lastLeftEncoderPosition = encoderLeft.getCurrentPosition();
        lastAuxEncoderPosition = encoderAux.getCurrentPosition();

        // calculate x, y, and theta (heading) deltas (robot perspective) since last measurement
        int dl = lastLeftEncoderPosition  - oldLeftCounter;
        int dr= lastRightEncoderPosition - oldRightCounter;
        int da = lastAuxEncoderPosition - oldAuxOdometryCounter;
        double dtheta = DEADWHEEL_MM_PER_TICK * (dr-dl) / DEADWHEEL_TRACKWIDTH; // should this be arctan?
        double dx = DEADWHEEL_MM_PER_TICK * (dl+dr) / 2.0;
        double dy = DEADWHEEL_MM_PER_TICK * (da - DEADWHEEL_FORWARD_OFFSET * (dr-dl) / DEADWHEEL_TRACKWIDTH);

        // update the x, y, and heading odometry counters
        xOdometryOdometryCounter += dx;
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
        xOdometryOdometryCounter = 0.0;
        yOdometryCounter = 0.0;
        headingOdometryCounter = 0.0;
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
     * Drive forward (reverse) while maintaining current heading and limiting sideways drift
     * @param distance Distance (MM) to move: + is forward, - is reverse
     * @param speed Speed factor to apply (should use defined constants)
     */
    public void drive(double distance, double speed) {

    }

    /**
     * Strafe left (right) while maintaining current heading and limiting forward/backward drift
     * @param distance Distance (MM) to move: + is left, - is right
     * @param speed Speed factor to apply (should use defined constants)
     */
    public void strafe(double distance, double speed) {

    }

    /**
     * Turn a relative angle while maintaining current position
     * @param angle Angle to rotate in Radians: + is counter-clockwise, - is clockwise
     * @param speed Speed factor to apply (should use defined constants)
     */
    public void turn(double angle, double speed) {

    }

    /* ----- High-level movement methods for autonomous motion ----- */

    /**
     * Move robot to specified field coordinate position (X, Y) in MM units
     * This method should be called from a LinerOpMode and implements its own
     * loop to cover the robots motion to the specified position.
     */
    public void moveToPosition(double x, double y, double speed) {

        /*
         * the x and y are in field coordinate system, so the first step is to convert them to
         * robot-oriented coordinates based on the current filed position and heading of the robot.
         */

        // Proportional controllers for X and Y power
        ProportionalController xController = new ProportionalController(PID_CONTROLLER_X_KP, x);
        ProportionalController yController = new ProportionalController(PID_CONTROLLER_Y_KP, y);

        /* loop until position is obtained or opMode is stopped */
        while(((LinearOpMode)myOpMode).opModeIsActive()) {

            /* update the current position */
            updateOdometry();

            /* check if we are close enough to the target */
            if (Math.abs(x - getPosX()) < MOVE_POSITION_TOLERANCE && Math.abs(y - getPosY()) < MOVE_POSITION_TOLERANCE) {
                break;
            }

            /* calculate s */
            double dx = x - getPosX();
            double dy = y - getPosY();

            /* convert deltas to robot-oriented X and Y errors for PID controller calculations */
            double h = getHeading();
            double errorX = dx * Math.cos(h) + dy * Math.sin(h);
            double errorY = -dx * Math.sin(h) + dy * Math.cos(h);


            /* set motor power through move method */
            //move(powerX, powerY, 0.0, false); // no yaw


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
            if (Math.abs(heading - getHeading()) < ROTATE_HEADING_TOLERANCE) {
                break;
            }

            /* measure error between current heading and target for PID controller calculations */
            double error = heading - getHeading();

            /* use PID controller to calculate yaw rate (power) */
            double derivative = (error - lastError) / timer.seconds(); // rate of change of the error
            integralSum += error * timer.seconds(); // sum of all error over time
            double power = PID_CONTROLLER_HEADING_KP * error + PID_CONTROLLER_HEADING_KI * integralSum + PID_CONTROLLER_HEADING_KD * derivative;

            /* set motor power through move method */
            //move(0.0, 0.0, power, speed); // no x or y motion

            /* update last error values */
            lastError = error;

            /* reset timer for next iteration */
            timer.reset();
        }
    }

}

/* ----- Closed-loop controller classes for use by mid and high level motion functions ----- */
/**
 * Proportional Controller class for computing motor power during autonomous motion
 */
class ProportionalController {

        // Proportional Gain
        private double Kp;

        // Desired position
        private double desiredPosition;

        // Constructor to set the gain and desired position
        public ProportionalController(double Kp, double desiredPosition) {
            this.Kp = Kp;
            this.desiredPosition = desiredPosition;
        }

        // Method to calculate the control output based on the current position
        public double calculate(double currentFieldPosition) {

            // Calculate the error
            double error = desiredPosition - currentFieldPosition;

            // Calculate the control output (proportional control)
            return Kp * error;

        }
}

/**
 * PID Controller class for computing motor power during autonomous motion
 */
class PIDController {

    // gains
    private double Kp;
    private double Ki;
    private double Kd;

    // Desired position
    private double desiredPosition;

    // tracking values
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private int lastTime = 0;

    // Constructor to set the gains and desired position
    public PIDController(double Kp, double Ki,double Kd,double desiredPosition) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.desiredPosition = desiredPosition;
    }

    // Method to calculate the control output based on the current position
    public double calculate(double currentFieldPosition) {

        // Calculate the error
        double error = desiredPosition - currentFieldPosition;

        // Get elapsed time (secs) since last calculation
        int currentTime = (int) System.currentTimeMillis() / 1000;
        int deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        // Update the integral sum
        integralSum += error * deltaTime;

        // Calculate the derivative and integral terms
        double derivative = (error - lastError) / deltaTime; // rate of change of the error
        lastError = error;

        // Calculate the control output (proportional control)
        return Kp * error + Ki * integralSum + Kd * derivative;
    }
}