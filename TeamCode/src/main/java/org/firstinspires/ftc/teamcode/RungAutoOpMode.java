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
@Autonomous(name="Right-Side (Rung) Autonomous", group="Competition",preselectTeleOp = "Two-controller Teleop")
//@Disabled
public class RungAutoOpMode extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();

    public void rungStrat() {
        //rotating the arm straight up
        if(opModeIsActive())
            robot.setArmRotation(0.17);

        //move towards the rung and extend arm at the same time
        if(opModeIsActive())
            robot.setArmExtension(1600);

        if(opModeIsActive())
            robot.forward(230, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        // rotate down to touch rung
        if(opModeIsActive())
            robot.setArmRotation(0.20);

        // let arm settle
        sleep(800);

        // lighten grip
        robot.closeClaw(false);

        //pull arm down to place the specimen on the rung (PROCEED WITH CAUTION)
        if(opModeIsActive())
            robot.setArmExtension(700, true, false);

        //open claw
        if(opModeIsActive())
            robot.openClaw(true);

        //retract arm
        if (opModeIsActive())
            robot.setArmExtension(0);

        //rotate arm fully downward
        if (opModeIsActive())
            robot.setArmRotation(0.35);

       //robot moves away from rung
        if(opModeIsActive())
            robot.forward(-230, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
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
            robot.forward(-200, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);
        //turn towards wall 2

        if(opModeIsActive())
            robot.turn(-(Math.PI/2), RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);
        //robot move towards the wall

        if(opModeIsActive())
            robot.forward(660, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
        //open claw for specimen on the floor

        if(opModeIsActive()) {
            robot.setArmRotation(0.3755);
            sleep(800);
        }
        //extend downwards

        robot.setArmExtension(1251);
        //close claw to capture specimen on the wall
        if(opModeIsActive())
            robot.closeClaw(true);
        //rotate arm upward
        if(opModeIsActive())
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
            robot.forward(240, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);
        //call the method "rungStrat"
        if(opModeIsActive())
            rungStrat();

        // Make sure robot stops and claw is open (teleop initialization default) before OpMode dies
        robot.stop();
        robot.openClaw(false);

    }
}
