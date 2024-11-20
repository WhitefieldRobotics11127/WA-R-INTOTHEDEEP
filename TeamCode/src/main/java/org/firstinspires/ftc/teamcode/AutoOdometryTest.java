// This file not meant for redistribution - copyright notice removed

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * A Linear OpMode with a simple linear route and turns for testing mid-level drive functions
 * in the RobotHardware class along with validation of odometry calibration and PID coefficient
 * settings.
 */

@Autonomous(name="Autonomous Drive/Odometry Test", group="Test")
//@Disabled

public class AutoOdometryTest extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    RobotHardware robot = new RobotHardware(this);

    // Declare OpMode members.
    final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // initialize all the hardware
        // NOTE: This resets the odometry counters to zero
        robot.init();

        // Set the initial field position of the robot
        robot.setFieldPosition(-1050,1130,0);

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

        // Move forward 2500 mm
        if (opModeIsActive())
            robot.forward(2500,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        // Strafe left 2000 mm
        if (opModeIsActive())
            robot.strafe(2000, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        // Rotate clockwise 90 degrees
        if (opModeIsActive())
            robot.turn(-Math.PI/2, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        // Move forward 2000 mm
        if (opModeIsActive())
            robot.forward(2000,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        // Rotate clockwise 90 degrees
        if (opModeIsActive())
            robot.turn(-Math.PI/2, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        // Move forward 2500 mm - Robot should end up back in the test square facing the opposite
        // direction from the start position. :)
        if (opModeIsActive())
            robot.forward(2500,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        // Stop the robot - just in case
        robot.stop();

        // Display the time elapsed and final field position in telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Field Position", "X: %.1f  Y: %.1f  Theta: %.1f",
                robot.getFieldPosX(), robot.getFieldPosY(), robot.getFieldHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}
