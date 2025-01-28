package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.openftc.apriltag.AprilTagPose;

/*
 * Autonomous OpMode for Bucket Side operation in 2024-2025 INTO THE DEEP season.
 */

@Autonomous(name="Bucket Auto Vision - Red", group="Competition", preselectTeleOp = "Two-controller Teleop")
//@Disabled

public class BucketAutoVisionRed extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public void dropInBucket() {
        //rotate arm upwards to field.
        if (opModeIsActive())
            robot.setArmRotation(0.12);

        //extend arm
        if (opModeIsActive())
            robot.setArmExtension(RobotHardware.ARM_EXTENSION_LIMIT_FULL, false, true);

        //approach bucket
        if (opModeIsActive())
            robot.forward(130, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //release claw to drop block and close claw
        if (opModeIsActive())
            robot.openClaw(false);

        //the Scoot!
        if (opModeIsActive())
            robot.forward(-130, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS );

        //retract the arm
        if (opModeIsActive())
            robot.setArmExtension(200);

        //rotate arm back
        if (opModeIsActive())
            robot.setArmRotation(.19);
    }
    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init(true);

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

        //Move off the alliance wall (nominal starting position up against the wall)
        if (opModeIsActive())
            robot.forward(250,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        /* This is an example of using the camera(s) and AprilTags to calculate a distance
         * for an upcoming move based on the numbers that were originally used in the autonomous
         * opmode.
         */
        // Measure the longitudinal (y) distance to the bucket wall using camera 1 and tag 16,
        // calculate distance to move towards bucket, then turn off the camera.
        // NOTE: Using the nominal starting position, this distance reads distance at 1600 mm. The
        // original distance of this move was 960 mm
        robot.switchCamera(1);
        double distanceToWall = robot.getLongitudinalDistanceToAprilTag(1, 16);
        double distanceToMove = 960;
        if (distanceToWall >  0)
            distanceToMove = (distanceToWall - 1600) - (1600 - 960);
        robot.switchCamera(0);

        //turn to face the bucket wall
        if (opModeIsActive())
            robot.turn(Math.PI/2,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        /* This is an example of using the distance sensor(s) to calculate a distance
         * for an upcoming move based on the numbers that were originally used in the autonomous
         * opmode.
         */
        // Measure the distance to the bucket wall using the front distance sensor, and
        // then calculate distance to move towards bucket.
        // NOTE: Using the nominal starting position, this distance reads distance at 1600 mm. The
        // original distance of this move was 960 mm
        //double distanceToWall = robot.getFrontDistance();
        //double distanceToMove = 960;
        //if (distanceToWall >  0)
        //    distanceToMove = (distanceToWall - 1600) - (1600 - 960);

        // move toward bucket wall
        if (opModeIsActive())
            robot.forward(distanceToMove,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        // rotate toward the bucket
        if (opModeIsActive())
            robot.turn(Math.PI/4,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        // call method to drop block in bucket
        if (opModeIsActive())
            dropInBucket();

        // backoff from bucket to clear samples on spikes
        if (opModeIsActive())
            robot.forward(-620,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //turn to face blue alliance wall
        // reduced from -3pi/4 (-135 degrees) to try and fix apparent overshoot
        if(opModeIsActive())
            robot.turn(-2.30, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        /* This is an example of using the Pose3D returned for an AprilTag from the vision processor
         * to measure error between where we are and where the robot is supposed to be to make
         * (possibly multiple, different) corrections before
         * proceeding.
         */
        // Get the pose of AprilTag 16 using camera 1, and calculate the error in heading and y
        // Nominal reading here is yaw of 0 and y = 850
        robot.switchCamera(1);
        AprilTagPoseFtc pose = robot.getAprilTagPose(1, 16);
        double headingError = (Math.PI / 2.0) - pose.yaw;
        double yError = 850 - pose.y;

        robot.switchCamera(0);

        //move to box
        if(opModeIsActive())
            robot.forward(850,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //turn to face box
        if(opModeIsActive())
            robot.turn(-Math.PI/2, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //go to box thing
        if (opModeIsActive())
            robot.forward(270,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //extend the arm
        if (opModeIsActive())
            robot.setArmExtension(1800);

        //lower arm to contact rung
        if (opModeIsActive())
            robot.setArmRotation(0.25);

        // Make sure robot stops and claw is open (teleop initialization default) before OpMode dies
        robot.stop();
        robot.openClaw(false);
    }
}
