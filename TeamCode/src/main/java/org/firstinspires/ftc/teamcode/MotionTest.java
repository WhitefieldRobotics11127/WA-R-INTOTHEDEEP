// This file not meant for redistribution - copyright notice removed

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Timer;

/*
 * OpMode to test/calibrate drivetrain and odometry
 */
@TeleOp(name= "Drivetrain test/calibration", group="Test")
public class MotionTest extends OpMode
{
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    // objects to save last gamepad state
    Gamepad lastGamepad1 = new Gamepad();
    Gamepad lastGamepad2 = new Gamepad();

    // Declare OpMode members.
    final private ElapsedTime runtime = new ElapsedTime();
    double speedFactor = RobotHardware.MOTOR_SPEED_FACTOR_NORMAL;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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

        // reset the odometry counters to 0
        robot.resetOdometryCounters();

        // Set the initial position of the robot on the field
        // NOTE: There should be a "testing" position for running this opmode that is marked on the
        // field (e.g., with gaffers tape) so that the robot can be placed in the same position each
        // time.
        robot.setFieldPosition(0, 0, 0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        // update the odometry for the robot
        robot.updateOdometry();

        // check whether the right bumper is pressed and reset the odometry counters to 0
        if (gamepad1.right_bumper && !lastGamepad1.right_bumper) {
            robot.resetOdometryCounters();
        }

        // check whether the x button is pressed and change the speed factor to davis speed
        if (gamepad1.x && !lastGamepad1.x) {
            speedFactor = RobotHardware.MOTOR_SPEED_FACTOR_DAVIS; // fly high fry guy
        }
        // check whether the b button is pressed and change the speed factor to precise speed
        if(gamepad1.b && !lastGamepad1.b) {
            speedFactor = RobotHardware.MOTOR_SPEED_FACTOR_PRECISE;
        }
        // check whether the a button is pressed and change the speed factor to normal speed
        if(gamepad1.a && !lastGamepad1.a) {
            speedFactor = RobotHardware.MOTOR_SPEED_FACTOR_NORMAL;
        }

        // if the d-pad buttons are presse, ignore the joystick input(s)
        if(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            if(gamepad1.dpad_up && !lastGamepad1.dpad_up) {
                robot.move(1, 0, 0, speedFactor);
            } else if(gamepad1.dpad_down && !lastGamepad1.dpad_down) {
                robot.move(-1, 0, 0, speedFactor);
            } else if(gamepad1.dpad_left && !lastGamepad1.dpad_left) {
                robot.move(0, 1, 0, speedFactor);
            } else if(gamepad1.dpad_right && !lastGamepad1.dpad_right) {
                robot.move(0, -1, 0, speedFactor);
            }
        } else {

            // Use left joystick to go forward & strafe, and right joystick to rotate.
            // Note that the robot.move() function takes values in FTC coordinate system values, where
            // +x is forward, +y is left, and +yaw is counter-clockwise rotation.
            double axial = -gamepad1.left_stick_y;  // pushing stick forward gives negative value
            double lateral = -gamepad1.left_stick_x;  // pushing stick left gives negative value
            double yaw = -gamepad1.right_stick_x;  // pushing stick left gives negative value

            // Send the power level to the wheels
            robot.move(axial, lateral, yaw, speedFactor);
        }

        // Save the current gamepad states
        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);

        // Show the elapsed game time and odometry information
        telemetry.addData("Status", "Running (%s)", runtime.toString());
        telemetry.addData("Raw Encoders", "Left: %d, Right: %d, Aux: %d", robot.lastLeftEncoderPosition, robot.lastRightEncoderPosition, robot.lastAuxEncoderPosition);
        telemetry.addData("Odometry", "x: %f mm, y: %f mm, hdg: %f rad", robot.getOdometryX(), robot.getOdometryY() , robot.getOdometryHeading());
        telemetry.addData("Field Position", "x: %f mm, y: %f mm, hdg: %f °", robot.getFieldPosX(), robot.getFieldPosY() , robot.getFieldHeading(AngleUnit.DEGREES));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        // Stop the robot
        robot.stop();

        // do final odometry update
        robot.updateOdometry();

        telemetry.addData("Status", "Stopped. Total Runtime: (%s)", runtime.toString());
        telemetry.addData("Odometry", "x: %f mm, y: %f mm, hdg: %f rad", robot.getOdometryX(), robot.getOdometryY() , robot.getOdometryHeading());
        telemetry.addData("Field Position", "x: %f mm, y: %f mm, hdg: %f °", robot.getFieldPosX(), robot.getFieldPosY() , robot.getFieldHeading(AngleUnit.DEGREES));
    }
}
