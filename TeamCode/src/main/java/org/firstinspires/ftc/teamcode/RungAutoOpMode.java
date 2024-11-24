package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Autonomous OpMode for Rung operation in 2024-2025 INTO THE DEEP season.
 */

@Autonomous(name="Right-Side (Rung) Autonomous", group="Competition")
//@Disabled

public class RungAutoOpMode extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();



    //@Override

    public void rungStrat() {
        //rotating the arm straight up
        if(opModeIsActive())
            robot.setArmRotation(0.39);
        //move towards the rung and extend arm at the same time
        if(opModeIsActive())
            robot.move(500, 0,0,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
        if(opModeIsActive())
            robot.setArmExtension(robot.ARM_EXTENSION_MAX);
        //pull arm down to place the specimen on the rung (PROCEED WITH CAUTION)
        if(opModeIsActive())
            robot.setArmExtension(2204);
        //open claw
        if(opModeIsActive())
            robot.openClaw(true);
        //retract arm
        if (opModeIsActive())
            robot.setArmExtension(0);
        //rotate arm fully downward
        if (opModeIsActive())
            robot.setArmRotation(0);

    }
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

        //move bot off the wall
        if(opModeIsActive())
            robot.move(100, 0, 0, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
        //call the method "rungStrat"
        if(opModeIsActive())
            rungStrat();
        //robot moves away from the box
        if(opModeIsActive())
            robot.move(-500, 0, 0, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
        //turn towards wall 2
        if(opModeIsActive())
            robot.turn(3*(Math.PI)/2, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
        //robot move towards the wall
        if(opModeIsActive())
            robot.move(870, 0, 0, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
        //open claw for specimen on the floor
        if(opModeIsActive())
            robot.openClaw(true);
        //extend downwards
        robot.setArmExtension(1469);
        //close claw to capture specimen on the wall
        if(opModeIsActive())
            robot.closeClaw(true);
        //retract arm back
        if(opModeIsActive())
            robot.setArmExtension(0);
        //turn back
        if(opModeIsActive())
            robot.turn(Math.PI,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
        //move back
        if(opModeIsActive())
            robot.move(870,0, 0, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
        //turn toward the rung
        if(opModeIsActive())
            robot.turn(3*((Math.PI)/2),RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
        //call the method "rungStrat"
        if(opModeIsActive())
            rungStrat();


    }
}
