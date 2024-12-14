package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Autonomous OpMode for Bucket Side operation in 2024-2025 INTO THE DEEP season.
 */

@Autonomous(name="Left-side (Bucket) Autonomous", group="Competition", preselectTeleOp = "Two-controller Teleop")
//@Disabled

public class BucketAutoOpMode extends LinearOpMode {

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

        //Move off the wall
        if (opModeIsActive())
            robot.forward(250,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //turn to face the bucket
        if (opModeIsActive())
            robot.turn(Math.PI/2,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        // move toward bucket wall
        if (opModeIsActive())
            robot.forward(960,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        // rotate toward the bucket
        if (opModeIsActive())
            robot.turn(Math.PI/4,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        // call method to drop block in bucket
        if (opModeIsActive())
            dropInBucket();

        // backoff from bucket to clear samples on spikes
        if (opModeIsActive())
            robot.forward(-620,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

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
