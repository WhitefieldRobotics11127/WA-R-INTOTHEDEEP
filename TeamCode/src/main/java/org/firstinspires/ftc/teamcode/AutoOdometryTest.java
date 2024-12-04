// This file not meant for redistribution - copyright notice removed

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * A Linear OpMode with a simple linear route and turns for testing mid-level drive functions
 * in the RobotHardware class along with validation of odometry calibration and PID coefficient
 * settings.
 */

@TeleOp(name="Autonomous Drive/Odometry Test", group="Test")
//@Disabled

public class AutoOdometryTest extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    RobotHardware robot = new RobotHardware(this);

    // Declare OpMode members.
    final ElapsedTime runtime = new ElapsedTime();
    Gamepad lastGamepad1 = new Gamepad(); //// save last gamepad state.

    @Override
    public void runOpMode() {

        // initialize all the hardware
        // NOTE: This resets the odometry counters to zero
        robot.init();

        // Update the status in telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Start the timer
        runtime.reset();

        // Perform autonomous motion commands
        // NOTE: Each mid-level drive command in the RobotHardware class has it's own loop
        // with a check for opModeIsActive(). But check here before each command to avoid
        // potentially leaving the robot in motion.

        // NOTE: These steps assume the robot is initially positioned in the test square
        // facing parallel to the red alliance wall.

        // Move forward 2500 mm and display odometry telemetry
        if (opModeIsActive()) {
            robot.forward(2000, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            odometryTelemetry();
        }

        // Strafe left 2000 mm
        if (opModeIsActive()) {
            robot.strafe(2000, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            odometryTelemetry();
        }

        // Rotate clockwise 90 degrees
        if (opModeIsActive()) {
            robot.turn(-Math.PI/2, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            odometryTelemetry();
        }

        // Move forward 2000 mm
        if (opModeIsActive()) {
            robot.forward(2000,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            odometryTelemetry();
        }

        // Rotate clockwise 90 degrees
        if (opModeIsActive()) {
            robot.turn(-Math.PI/2, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            odometryTelemetry();
        }

        // Move forward 2500 mm - Robot should end up back in the test square facing the opposite
        // direction from the start position. :)
        if (opModeIsActive()) {
            robot.forward(2000,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            odometryTelemetry();
        }

        // Stop the robot - just in case
        robot.stop();

        // Enter a teleop period to allow for manual invocation of mid-level drive functions
        while (opModeIsActive()) {

            // call mid-level motion function to Move the robot about a foot in the direction
            // of the d-pad button pressed or rotate 90 degrees in the direction of the bumper
            // pressed. This is for testing odometry and PID tuning.
            if(gamepad1.dpad_up && !lastGamepad1.dpad_up)
                robot.forward(300, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            else if(gamepad1.dpad_down && !lastGamepad1.dpad_down)
                robot.forward(-300, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            else if(gamepad1.dpad_left && !lastGamepad1.dpad_left)
                robot.strafe(300, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            else if(gamepad1.dpad_right && !lastGamepad1.dpad_right)
                robot.strafe(300, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            else if(gamepad1.left_bumper && !lastGamepad1.left_bumper)
                robot.turn(Math.PI / 2, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
            else if(gamepad1.right_bumper && !lastGamepad1.right_bumper)
                robot.turn(Math.PI / 2, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

            // Display odometry telemetry
            odometryTelemetry();

            // Save the current gamepad state
            lastGamepad1.copy(gamepad1);
        }

        // stop the robot (just in case)
        robot.stop();

        // Display the time elapsed and final field position in telemetry
        telemetry.addData("Status", " Stopped. Run Time: %s", runtime.toString());
        telemetry.update();
    }

    void odometryTelemetry() {
        // Display odometry telemetry
        telemetry.addData("Status", "Running (%s)", runtime.toString());
        telemetry.addData("Odometry", "X: %.1f  Y: %.1f  Theta: %.3f",
                robot.getOdometryX(), robot.getOdometryY(), robot.getOdometryHeading());
        telemetry.update();
    }
}
