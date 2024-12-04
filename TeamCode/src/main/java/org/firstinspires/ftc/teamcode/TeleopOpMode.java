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
@TeleOp(name= "Two-controller Teleop", group="Competition")
//@Disabled
public class TeleopOpMode extends OpMode
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
    double loganButton = RobotHardware.MOTOR_SPEED_FACTOR_PRECISE;
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

        // ***** Handle movement control from Gamepad1 *****

        // Set the current speedFactor from button presses on the first gamepad:
        // A: Normal speed
        // B: Precise speed
        // X: Davis speed
        // Y: D-Pad speed setter
        if(gamepad1.a && !lastGamepad1.a) {
            speedFactor = RobotHardware.MOTOR_SPEED_FACTOR_NORMAL;
        }
        else if (gamepad1.x && !lastGamepad1.x) {
            speedFactor = RobotHardware.MOTOR_SPEED_FACTOR_DAVIS; // fly high fry guy
        }
        else if(gamepad1.b && !lastGamepad1.b) {
            speedFactor = RobotHardware.MOTOR_SPEED_FACTOR_PRECISE;
        }
        else if(gamepad1.y) {
            loganButton = speedFactor;
        }
        else {
            System.out.println("Logan probably messed up the code.");
        }
        // If d-pad input provided, ignore joystick input(s)
        if(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {

            // Move the robot in the direction of the d-pad button pressed at 1/2 precision speed
            if(gamepad1.dpad_up && !lastGamepad1.dpad_up) {
                robot.move(1, 0, 0, loganButton * 0.5);
            } else if(gamepad1.dpad_down && !lastGamepad1.dpad_down) {
                robot.move(-1, 0, 0, loganButton * 0.5);
            } else if(gamepad1.dpad_left && !lastGamepad1.dpad_left) {
                robot.move(0, 1, 0, loganButton * 0.5);
            } else if(gamepad1.dpad_right && !lastGamepad1.dpad_right) {
                robot.move(0, -1, 0, loganButton * 0.5);
            }


        } else {
            // ignore right stick inputs if bumpers are pressed
            if (gamepad1.right_bumper || gamepad1.left_bumper){
                //rotate the bot
               if (gamepad1.right_bumper)
                   robot.move(0,0, 1, loganButton * 0.5);
               else if(gamepad1.left_bumper)
                   robot.move(0, 0, -1, loganButton * 0.5);
            }
            else {
                // Use left joystick to go forward & strafe, and right joystick to rotate.
                // NOTE: the robot.move() function takes values in FTC coordinate system values, where
                // +x is forward, +y is left, and +yaw is counter-clockwise rotation.
                double axial = -gamepad1.left_stick_y;  // pushing stick forward gives negative value
                double lateral = -gamepad1.left_stick_x;  // pushing stick left gives negative value
                double yaw = -gamepad1.right_stick_x;  // pushing stick left gives negative value
                robot.move(axial, lateral, yaw, speedFactor);
            }
        }

        // ***** Handle arm and claw control from Gamepad2 *****

        // Right trigger controls the claw servo
        double servoPosition;
        if (gamepad2.right_trigger > 0.4)
            robot.closeClaw((gamepad2.left_trigger > 0.4));
        else
            robot.openClaw((gamepad2.left_trigger > 0.4));

        // If the arm extension motor is busy extending arm to a position, check progress
        // and reset flag if finished
        if (inArmExtensionOperation) {
            if (!robot.isArmExtensionBusy()) {
                robot.stopArmExtension();
                inArmExtensionOperation = false;
            }
        }
        else {

            // If the arm becomes outside of limits, e.g., as indicated by loud clicking sound of
            // the belt drive, use the B button to suspend the limits and then fully retract (home)
            // the arm. Then use the A button to reset the limit values (ARM_EXTENSION_MAX and
            // ARM_EXTENSION_MIN) from the fully retracted (0) position.
            if (gamepad2.a && !lastGamepad2.a) {
                robot.resetArmLimits();
            }
            else if (gamepad2.b && !lastGamepad2.b) {
                robot.suspendArmLimits();
            }

            // If a particular position for the arm extension is desired, use the RUN_TO_POSITION
            // capability of the motor/encoder to extend/retract the arm to the desired position
            // in the background.
            if (gamepad2.left_bumper && !lastGamepad2.left_bumper ||
                    gamepad2.right_bumper && !lastGamepad2.right_bumper ||
                    gamepad2.dpad_up && !lastGamepad2.dpad_up ||
                    gamepad2.dpad_down && !lastGamepad2.dpad_down) {

                // if right bumper is pressed, fully extend the arm. This is a separate operation
                // because the limits change based on whether the current rotational position of
                // the arm is considered vertical.
                if (gamepad2.right_bumper)
                    robot.extendArmToLimit();
                // if left bumper is pressed, fully retract the arm
                else if (gamepad2.left_bumper)
                    robot.extendArmToPosition(robot.ARM_EXTENSION_MIN);
                // otherwise, move the arm extension by +/- 600 ticks
                else {
                    int pos = robot.getArmExtension();
                    if (gamepad2.dpad_up)
                        pos += 600;
                    else if (gamepad2.dpad_down)
                        pos -= 600;

                    // Set the arm extension to the specified position
                    robot.extendArmToPosition(pos);
                }

                // set flag to indicate that the arm extension motor is in a RUN_TO_POSITION operation
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
