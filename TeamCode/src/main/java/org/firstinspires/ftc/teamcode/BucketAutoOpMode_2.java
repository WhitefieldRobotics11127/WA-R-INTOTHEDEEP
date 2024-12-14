package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Autonomous OpMode for Bucket Side operation in 2024-2025 INTO THE DEEP season.
 */

@Autonomous(name="Left-side (Bucket) Autonomous (2)", group="Competition", preselectTeleOp = "Odometry-enabled Teleop")
//@Disabled

public class BucketAutoOpMode_2 extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public void dropInBucket() {

        //rotate arm upwards to field.
        if (opModeIsActive())
            robot.setArmRotation(0.11);

        //extend arm
        if (opModeIsActive())
            robot.setArmExtension(RobotHardware.ARM_EXTENSION_LIMIT_FULL);

        //approach bucket
        if (opModeIsActive())
            robot.forward(80, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //release claw to drop block and close claw
        if (opModeIsActive())
            robot.openClaw(true);

        //the Scoot!
        if (opModeIsActive())
            robot.forward(-80, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS );

        //retract the arm
        if (opModeIsActive())
            robot.setArmExtension(200);

        //rotate arm down
        if (opModeIsActive())
            robot.setArmRotation(0.35);
    }
    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        robot.closeClaw(true);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Reset the runtime timer
        runtime.reset();

        // Check if the opMode is still active (end of autonomous period or driver presses STOP)
        // before each command

        /***** Place pre-loaded sample in bucket *****/
        //Move off the wall
        if (opModeIsActive())
            robot.forward(250,RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);

        //turn to face the audience wall
        // Increased from PI/2
        if (opModeIsActive())
            robot.turn(Math.PI / 2,RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);

        // move toward bucket wall
        if (opModeIsActive())
            robot.forward(960,RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);

        // rotate toward the bucket
        // Increased from PI/4 (45 degrees)
        if (opModeIsActive())
            robot.turn(.80,RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);

        // call method to drop block in (high) bucket
        if (opModeIsActive())
            dropInBucket();

        /***** Retrieve second sample and place in bucket *****/
        // backoff from bucket to align center of rotation with outer spike
        if (opModeIsActive())
            robot.forward(-250,RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);

        //turn to face blue wall (sample location)
        if(opModeIsActive())
            robot.turn(-3 * Math.PI / 4, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        // We may want to correct position using AprilTag here
        //if (opModeIsActive())
        //    robot.moveToPositionUsingAprilTag(-1301.6, -1083.0, 1.607, 1, 16, RobotHardware.MOTOR_SPEED_FACTOR_PRECISE);

        // pickup sample from spike
        if (opModeIsActive()) {
            robot.setArmRotation(0.40); // rotate arm down to grip position
            robot.setArmExtension(1300); // extend arm to grip position
            robot.closeClaw(true); // close claw to tight grip
            robot.setArmRotation(0.27); // rotate arm up
            robot.setArmExtension(200); // retract arm for easier movement
        }

        // Reverse the steps above to move back to bucket
        // move back from grip point of second sample
        //if (opModeIsActive())
        //    robot.forward(0,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //turn back to face bucket
        if(opModeIsActive())
            robot.turn(3 * Math.PI / 4, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        // move back to bucket drop position
        if (opModeIsActive())
            robot.forward(250,RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);

        // drop second sample in high bucket
        if (opModeIsActive())
            dropInBucket();

        /***** End Game *****/
        // backoff from bucket to clear samples on spikes
        if (opModeIsActive())
            robot.forward(-620,RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);

        //turn to face blue wall
        // reduced from -3pi/4 (-135 degrees) to try and fix apparent overshoot
        if(opModeIsActive())
            robot.turn(-2.30, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //move to box
        if(opModeIsActive())
            robot.forward(850,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //turn to face box
        if(opModeIsActive())
            robot.turn(-Math.PI/2, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //go to box thing
        if (opModeIsActive())
            robot.forward(315,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //extend the arm
        if (opModeIsActive())
            robot.setArmExtension(1800);

        //lower arm to contact rung
        if (opModeIsActive())
            robot.setArmRotation(0.26);

    }
}
