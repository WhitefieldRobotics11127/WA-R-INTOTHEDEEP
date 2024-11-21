// This file not meant for redistribution - copyright notice removed

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

/*
 * Primary Teleop OpMode for the 2024-2025 INTO THE DEEP competition robot.
 * This OpMode is setup for two-controller (gamepad1 and gamepad2) operation of the robot and
 * does not support any
 */
@TeleOp(name= "Odometry-enabled Teleop", group="Test")
//@Disabled
public class TeleopOdometry extends OpMode
{
    // Create a RobotHardware object to be used to access robot hardware.
    // All of the methods and properties of the RobotHardware class should be accessed via the
    // "robot.", e.g., "robot.move(x, y, yaw, speed)," etc. All "constants" (static variables)
    // defined in the RobotHardware class must be accessed via the "RobotHardware." prefix, e.g.,
    // "RobotHardware.MOTOR_SPEED_FACTOR_NORMAL," etc.
    // NOTE: This does not apply to ARM_LIMIT_MIN and ARM_LIMIT_MAX, which are accessed via the
    // "robot." prefix, e.g., "robot.ARM_LIMIT_MIN" and "robot.ARM_LIMIT_MAX," because these values
    // are not static and may change when the arm extension motor encoder position is reset.
    RobotHardware robot = new RobotHardware(this);

    // Gamepad objects to save last gamepad state. These can be used to, e.g., compare button states
    // to previous states to tie discrete actions and toggling states to button presses and releases
    // instead "thrashing" on long button presses that span multiple loop() calls.
    Gamepad lastGamepad1 = new Gamepad();
    Gamepad lastGamepad2 = new Gamepad();

    // Declare OpMode members. The elapsed time is one possible value to display in the telemetry
    // data on the driver station.
    final private ElapsedTime runtime = new ElapsedTime();

    // Speed factor for the robot motors. This is to be used as a current setting that can be
    // changed by button presses on the gamepad and then passed to the move() method of the robot
    // hardware class. The speed factor is used to scale the power levels sent to the motors to
    // control the speed of the robot. The speed factor can be set to one of three values:
    //  - RobotHardware.MOTOR_SPEED_FACTOR_NORMAL: Normal safe speed
    //  - RobotHardware.MOTOR_SPEED_FACTOR_PRECISE: Slow (precise) speed for fine control and odometry tracking
    //  - RobotHardware.MOTOR_SPEED_FACTOR_DAVIS: "Sprint" (fast) speed
    // This may also be displayed in telemetry data on the driver station.
    double speedFactor = RobotHardware.MOTOR_SPEED_FACTOR_NORMAL;

    // A hashmap to store the speed factor names for display in telemetry data on the driver station.
    static final private HashMap<Double, String> speedFactorNames;
    static {
        speedFactorNames = new HashMap<Double, String>();
        speedFactorNames.put(RobotHardware.MOTOR_SPEED_FACTOR_NORMAL, "Normal");
        speedFactorNames.put(RobotHardware.MOTOR_SPEED_FACTOR_PRECISE, "Precise");
        speedFactorNames.put(RobotHardware.MOTOR_SPEED_FACTOR_DAVIS, "Davis");
    }

    // Flag to indicate that the arm extension motor is in a RUN_TO_POSITION operation. This allows
    // a single button press to extend/retract the arm to a specific position without tying up the
    // thread in the loop() function waiting for the arm to reach the desired position.
    boolean inArmExtensionOperation = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the robot hardware through the RobotHardware class.
        robot.init();

        // We need a way to retrieve the current Field Position from the Autonomous OpMode and
        // set it here.
        // NOTE: We need to set it to something because the get methods will fail if the object
        // is not initialized. Should be fixed in RobotHardware class at some point.
        robot.setFieldPosition(0, 0, 0);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Hardware Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

        // reset the runtime counter
        runtime.reset();

        // Reset the odometry counters to 0.
        robot.resetOdometryCounters();

        // If we are wanting to set the robot's position on the field, we can do that here with a
        // call to robot.setFieldPosition(). However, we really wouldn't know the robot's position
        // on the field from the prior Autonomous OpMode, so there needs to be some communication
        // of the robot's final state (including field position) from the Autonomous OpMode to this
        // Teleop OpMode. Use of a Singleton class to store the robot's field position and/or other
        // end-state values is one way to do this.
        //robot.setFieldPosition(0, 0, 0);

        // ***** What's the current extension of the arm? We need some way to determine what the
        // encoder value is for the arm's extension motor before robot.init() because it is going
        // to be reset to zero on init. *****

        // Setup the initial telemetry display for the driving team captain
        telemetry.addData("Status", "Running (%s)", runtime.toString());
        telemetry.addData("Speed Factor", speedFactorNames.get(speedFactor));
        telemetry.addData("Arm Rotation Position", robot.getArmRotation());
        telemetry.addData("Arm Extension Position", robot.getArmExtension());
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        // update the odometry for the robot
        robot.updateOdometry();

        // ***** Handle movement control from Gamepad1 *****

        // Reset odometry to zero if the Y button is pressed
        if (gamepad1.y && !lastGamepad1.y) {
            robot.resetOdometryCounters();
        }

        // Set the current speedFactor from button presses on the first gamepad:
        // A: Normal speed
        // B: Precise speed
        // X: Davis speed
        if(gamepad1.a && !lastGamepad1.a) {
            speedFactor = RobotHardware.MOTOR_SPEED_FACTOR_NORMAL;
        }
        else if (gamepad1.x && !lastGamepad1.x) {
            speedFactor = RobotHardware.MOTOR_SPEED_FACTOR_DAVIS; // fly high fry guy
        }
        else if(gamepad1.b && !lastGamepad1.b) {
            speedFactor = RobotHardware.MOTOR_SPEED_FACTOR_PRECISE;
        }

        // If d-pad input provided, ignore joystick input(s)
        if(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {

            // Move the robot in the direction of the d-pad button pressed at 1/2 precision speed
            if(gamepad1.dpad_up && !lastGamepad1.dpad_up) {
                robot.move(1, 0, 0, RobotHardware.MOTOR_SPEED_FACTOR_PRECISE * 0.5);
            } else if(gamepad1.dpad_down && !lastGamepad1.dpad_down) {
                robot.move(-1, 0, 0, RobotHardware.MOTOR_SPEED_FACTOR_PRECISE * 0.5);
            } else if(gamepad1.dpad_left && !lastGamepad1.dpad_left) {
                robot.move(0, 1, 0, RobotHardware.MOTOR_SPEED_FACTOR_PRECISE * 0.5);
            } else if(gamepad1.dpad_right && !lastGamepad1.dpad_right) {
                robot.move(0, -1, 0, RobotHardware.MOTOR_SPEED_FACTOR_PRECISE * 0.5);
            }

        } else {

            // Use left joystick to go forward & strafe, and right joystick to rotate.
            // NOTE: the robot.move() function takes values in FTC coordinate system values, where
            // +x is forward, +y is left, and +yaw is counter-clockwise rotation.
            double axial = -gamepad1.left_stick_y;  // pushing stick forward gives negative value
            double lateral = -gamepad1.left_stick_x;  // pushing stick left gives negative value
            double yaw = -gamepad1.right_stick_x;  // pushing stick left gives negative value
            robot.move(axial, lateral, yaw, speedFactor);
        }

        // ***** Handle arm and claw control from Gamepad2 *****

        // Right trigger controls the claw servo
        if (gamepad2.right_trigger > 0.4 && lastGamepad2.right_trigger <= 0.4)
            robot.closeClaw(false);
        else if (gamepad2.right_trigger < 0.4 && lastGamepad2.right_trigger >= 0.4)
            robot.openClaw(false);

        // If the arm extension motor is busy extending arm to a position, check progress
        // and reset flag if finished
        if (inArmExtensionOperation) {
            if (!robot.isArmExtensionBusy()) {
                robot.stopArmExtension();
                inArmExtensionOperation = false;
            }
        }
        else {

            // Reset the encoder value and limit values (ARM_EXTENSION_MAX and ARM_EXTENSION_MIN)
            // on the arm extension motor depending on position of arm. This is used if the arm
            // extension motor overruns full extension or retraction as indicated by the loud
            // clicking sound of the belt drive. The associated reset button should be pressed
            // based on the whether the overrun occurred during extension or retraction:
            //  - A: Reset encoder position on full retraction
            //  - B: Reset encoder position on full extension
            if (gamepad2.a && !lastGamepad2.a) {
                robot.resetArmLimitsRetracted();
            }
            else if (gamepad2.b && !lastGamepad2.b) {
                robot.resetArmLimitsExtended();
            }

            // If a particular position for the arm extension is desired, set it using the
            // setArmExtension method
            if (gamepad2.left_bumper && !lastGamepad2.left_bumper ||
                    gamepad2.right_bumper && !lastGamepad2.right_bumper ||
                    gamepad2.dpad_up && !lastGamepad2.dpad_up ||
                    gamepad2.dpad_down && !lastGamepad2.dpad_down) {

                int pos = robot.getArmExtension();
                // bumpers move to the limits, dpad up and down move the arm extension by 300 ticks
                if (gamepad2.left_bumper)
                    pos = robot.ARM_EXTENSION_MIN;
                else if (gamepad2.right_bumper)
                    pos = robot.ARM_EXTENSION_MAX;
                else if (gamepad2.dpad_up)
                    pos += 600;
                else if (gamepad2.dpad_down)
                    pos -= 600;

                // Set the arm extension to the specified position
                robot.extendArmToPosition(pos);
                inArmExtensionOperation = true;
            }

            // otherwise, get the arm movement from the left stick
            else {

                // Left stick Y controls the arm extension.
                robot.extendArm(-gamepad2.left_stick_y);
            }
        }

        // Right stick Y controls the arm rotation using a aircraft yoke pitch paradigm: pull back
        // to raise the arm and push forward to lower the arm.
        // NOTE: The rotateArm() method takes positive values to lower the arm and negative values
        // to raise the arm. This is so that the power applied for the rotation tracks with the
        // values returned from the rotation position sensor (potentiometer).
        robot.rotateArm(-gamepad2.right_stick_y);

        // Save the current gamepad states
        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);

        // Update the telemetry information
        telemetry.addData("Status", "Running (%s)", runtime.toString());
        telemetry.addData("Speed Factor", speedFactorNames.get(speedFactor));
        telemetry.addData("Odometry", "X: %.1f  Y: %.1f  Theta: %.3f",
                robot.getOdometryX(), robot.getOdometryY(), robot.getOdometryHeading());
        telemetry.addData("Arm Rotation Position", robot.getArmRotation());
        telemetry.addData("Arm Extension Position", robot.getArmExtension());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // Stop the robot
        robot.stop();

        telemetry.addData("Status", "Stopped. Total Runtime: (%s)", runtime.toString());
    }
}
