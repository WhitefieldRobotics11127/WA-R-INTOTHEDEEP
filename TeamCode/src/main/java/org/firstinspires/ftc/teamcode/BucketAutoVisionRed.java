package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        // Measure the longitudinal (y) distance to the bucket wall using left side camera (Camera 2)
        // and tag 16 and then calculate distance to move towards bucket.
        // NOTE: Using the nominal starting position, this distance roughly reads distance 1197 mm.
        // The original distance of this move was 960 mm
        // NOTE: This is the difference between Red and Blue. For Red alliance, the left side camera
        // (Camera 1) will be looking at tag ID 16. For Blue alliance, it's looking at tag ID 13
        // NOTE: The getLongitudinalDistanceToAprilTag switches the specified camera on before
        // taking the measurements and then disables the camera afterwards, so no need to do that
        // here
        double distanceToMove = 960;
        double distanceToWall = robot.getLongitudinalDistanceToAprilTag(2, 16);
        // add telemetry for debugging
        telemetry.addData("Distance to Wall", "%6.1f mm", distanceToWall);
        telemetry.update();
        if (distanceToWall >  0)
            distanceToMove = 960.0 - (1197.0 - distanceToWall);

        //turn to face the bucket wall
        if (opModeIsActive())
            robot.turn(Math.PI/2,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

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
        if(opModeIsActive())
            robot.turn(-3 * Math.PI / 4, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //move to box
        if(opModeIsActive())
            robot.forward(850,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //turn to face box
        if(opModeIsActive())
            robot.turn(-Math.PI/2, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        /* This is an example of using the distance sensor(s) to measure a distance after a move
         * to calculate the next move
         */
        // Measure the distance to the bucket wall using the rear distance sensor, and
        // then calculate the distance to move towards the submersible.
        // NOTE: Using the nominal opmode odometry, after the turn this distance reads roughly
        // 580 mm. The original move amount was 250
        distanceToWall = robot.getRearDistance();
        distanceToMove = 250 - (distanceToWall - 580);
        // add telemetry for debugging
        telemetry.addData("Distance to Wall", "%6.1f mm", distanceToWall);
        telemetry.update();

        //go to box thing
        if (opModeIsActive())
            robot.forward(distanceToMove,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

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
