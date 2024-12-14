package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Autonomous OpMode for Rung operation in 2024-2025 INTO THE DEEP season.
 */
/*
The idea of this Class is to have a backup just in case we are not on the left.
 */
@Autonomous(name="Right-Side (Rung) Autonomous (2)", group="Test", preselectTeleOp = "Odometry-enabled Teleop")
//@Disabled
public class RungAutoOpMode_2 extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    public void rungStrat() {
        //rotating the arm straight up
        if(opModeIsActive())
            robot.setArmRotation(0.19);
        //move towards the rung and extend arm at the same time
        if(opModeIsActive())
            robot.setArmExtension(2200);
        if(opModeIsActive())
            robot.forward(40, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
        //rotate the arm downward
        if(opModeIsActive())
            robot.setArmRotation(0.25);
        //pull arm down to place the specimen on the rung (PROCEED SLOWLY)
        if(opModeIsActive())
            robot.setArmExtension(1100);
        //open claw
        if(opModeIsActive())
            robot.openClaw(true);
        //retract arm
        if (opModeIsActive())
            robot.setArmExtension(0);
        //rotate arm fully downward
        if (opModeIsActive())
            robot.setArmRotation(RobotHardware.ARM_ROTATION_MAX);
//robot moves away from the box
        if(opModeIsActive())
            robot.forward(-40, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
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

        //move bot off the wall
        if(opModeIsActive())
            robot.forward(100, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);
        //strafe left
        if(opModeIsActive())
            robot.strafe(250, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);
        if(opModeIsActive())
            robot.forward(250, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);
        //call the method "rungStrat"
        if(opModeIsActive())
            rungStrat();
        //go to wall
        if(opModeIsActive())
            robot.forward(-225, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);
        //turn towards wall 2
        if(opModeIsActive())
            robot.turn(-(Math.PI/2), RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);
        //robot move towards the wall
        if(opModeIsActive())
            robot.forward(625, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);
        //open claw for specimen on the floor
        robot.setArmRotation(0.377);
        //extend downwards
        robot.setArmExtension(1182);
        //close claw to capture specimen on the wall
        if(opModeIsActive())
            robot.closeClaw(true);
        //rotate arm upward
        robot.setArmRotation(0.25);
        //retract arm back
        if(opModeIsActive())
            robot.setArmExtension(500);
        //move back
        if(opModeIsActive())
            robot.forward(-820, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);
        //turn toward the rung
        if(opModeIsActive())
            robot.turn(Math.PI/2,RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);
        //move forward toward the rung
        if(opModeIsActive())
            robot.forward(250, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);
        //call the method "rungStrat"
        if(opModeIsActive())
            rungStrat();


    }
}
