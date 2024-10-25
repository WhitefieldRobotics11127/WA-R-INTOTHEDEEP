// This file not meant for redistribution - copyright notice removed

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * OpMode to test/calibrate drivetrain and odometry
 */
@TeleOp(name= "Drivetrain test/calibration", group="Test")
public class MotionTest extends OpMode
{
    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    // Declare OpMode members.
    final private ElapsedTime runtime = new ElapsedTime();

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

        runtime.reset();

        // reset the odometry counters to 0
        // We may want to add a gamepad button test to reset the odometry counters to 0
        robot.resetOdometryCounters();

        // Set the initial position of the robot on the field
        // NOTE: There should be a "testing" position for running this opmode that is marked on the
        // field (e.g., with gaffers tape) so that the robot can be placed in the same position each

        robot.setFieldPosition(0, 0, 0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        // update the odometry for the robot
        robot.updateOdometry();

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

        // Send the power level to the wheels
        robot.move(axial, lateral, yaw, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Running (%s)", runtime.toString());
        telemetry.addData("Odometry", "x: %f mm, y: %f mm, hdg: %f °", robot.getOdometryX(), robot.getOdometryY() , robot.getOdometryHeading());
        telemetry.addData("Field Position", "x: %f mm, y: %f mm, hdg: %f °", robot.getPosX(), robot.getPosY() , robot.getHeading(AngleUnit.DEGREES));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Stop the robot
        robot.move(0, 0, 0, 0);

        telemetry.addData("Status", "Stopped. Total Runtime: (%s)", runtime.toString());
    }
}
