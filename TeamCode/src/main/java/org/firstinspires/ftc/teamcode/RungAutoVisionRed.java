package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

/*
 * Autonomous OpMode for Rung operation in 2024-2025 INTO THE DEEP season.
 */
/*
The idea of this Class is to have a backup just in case we are not on the left.
 */
@Autonomous(name="Run Auto Vision Red", group="Competition",preselectTeleOp = "Two-controller Teleop")
//@Disabled
public class RungAutoVisionRed extends LinearOpMode {

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
        sleep(700);

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
            robot.setArmExtension(50);

        //rotate arm fully downward
        if (opModeIsActive())
            robot.setArmRotation(0.35);

       //robot moves away from rung
        if(opModeIsActive())
            robot.forward(-230, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
    }
    @Override

    public void runOpMode() {

        // initialize all the hardware and enable vision
        robot.init(true );

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

        /*
        * Is any of the following code necessary? Can we just start at a position where we can
        * move forward?

        //move bot off the wall
        if(opModeIsActive())
            robot.forward(100, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);

        // Measure the longitudinal (y) distance to the specimen wall using right side camera
        // (Camera 1) and tag 14 and then calculate distance to align with submersible hangin
        // position.
        // NOTE: Using the nominal starting position, this distance roughly reads distance 1197 mm.
        // The original distance of this move was 250 mm
        // NOTE: This is the difference between Red and Blue. For Red alliance, the right side camera
        // (Camera 1) will be looking at tag ID 14. For Blue alliance, it's looking at tag ID 11
        // NOTE: The getLongitudinalDistanceToAprilTag switches the specified camera on before
        // taking the measurements and then disables the camera afterwards, so no need to do that
        // here
        double distanceToMove = 250;
        double distanceToWall = robot.getLongitudinalDistanceToAprilTag(1, 14);
        // add telemetry for debugging
        telemetry.addData("Distance to Wall", "%6.1f mm", distanceToWall);
        telemetry.update();
        if (distanceToWall >  0)
            distanceToMove = 250 - (1197.0 - distanceToWall);

        //strafe left
        if(opModeIsActive())
            robot.strafe(distanceToMove, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);

        //move towards the submersible
        if(opModeIsActive())
            robot.forward(250, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);
        */

        // Alternative "Move Off The Wall" and "Move Towards Submersible" code without the strafe
        if(opModeIsActive())
            robot.forward(350, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //call the method "rungStrat"
        if(opModeIsActive())
            rungStrat();

        // Measure the pose of AprilTag ID 14 using the right side camera and then determine the
        // distance to move back towards the alliance wall, amount to yaw to face the specimen, and
        // distance to move forward to pick up the specimen.
        // NOTE: Using the nominal position after hanging the first specimen, the distance to the
        // wall (y) reads roughly 1467 mm and the X offset reads roughly -117mm. The yaw should be
        // 0.
        // The original distance to move back is 200mm, the yaw is 90 degrees (Pi/2 radians), and
        // the distance to pickup point for second specimen is 635mm
        double distanceToBackup = 200;
        double yawToSpecimen = Math.PI/2;
        double distanceToMoveToSpecimen = 635;
        AprilTagPoseFtc tag14Pose = robot.getAprilTagPose(1, 14);
        if (tag14Pose != null) {
            telemetry.addData("AprilTag 14 Detected", "");
            telemetry.addLine(String.format("Y: %6.1f mm, X: %6.1f mm, Yaw: %1.3f rad",
                    tag14Pose.y,
                    tag14Pose.x,
                    tag14Pose.yaw
            ));
            telemetry.update();
            distanceToBackup = 200 - (-117 - tag14Pose.x);
            yawToSpecimen = Math.PI/2 - tag14Pose.yaw;
            distanceToMoveToSpecimen = 635 - (1467 - tag14Pose.y);
        }
        else {
            telemetry.addData("AprilTag 14 Not Detected", "");
            telemetry.update();
        }

        //backup to the wall
        if(opModeIsActive())
            robot.forward(-distanceToBackup, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);

        //turn specimen wall
        if(opModeIsActive())
            robot.turn(-yawToSpecimen, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //robot move towards the wall
        if(opModeIsActive())
            robot.forward(distanceToMoveToSpecimen, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);

        //lower claw to pick up specimen
        if(opModeIsActive()) {
            robot.setArmRotation(0.3755);
            sleep(800);
        }

        //extend arm to specimen
        robot.setArmExtension(1251);

        //close claw to capture specimen
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
            robot.turn(Math.PI/2,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        // Measure the pose of AprilTag ID 14 again to see if any corrections are necessary before
        // moving towards the submersible and running the hang method
        // NOTE: Extrapolation from the nominal position after hanging the first specimen, the
        // the X offset should roughly -317 mm (-117 + -200). The yaw should be 0.
        // The original distance to move forward is 200mm, and the yaw should be corrected to 0
        double distanceToMoveForward = 200;
        double yawCorrection = 0;
        tag14Pose = robot.getAprilTagPose(1, 14);
        if (tag14Pose != null) {
            telemetry.addData("AprilTag 14 Detected", "");
            telemetry.addLine(String.format("Y: %6.1f mm, X: %6.1f mm, Yaw: %1.3f rad",
                    tag14Pose.y,
                    tag14Pose.x,
                    tag14Pose.yaw
            ));
            telemetry.update();
            distanceToMoveForward = 200 + (-317 - tag14Pose.x);
            yawCorrection = tag14Pose.yaw;
        }
        else {
            telemetry.addData("AprilTag 14 Not Detected", "");
            telemetry.update();
        }

        // correct heading towards submersible, if necessary (more than 2 degrees off)
        if (opModeIsActive() && Math.abs(yawCorrection) > RobotHardware.HEADING_TOLERANCE)
            robot.turn(yawCorrection, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

        //move forward toward the rung
        if(opModeIsActive())
            robot.forward(distanceToMoveForward, RobotHardware.MOTOR_SPEED_FACTOR_NORMAL);

        //call the method "rungStrat"
        if(opModeIsActive())
            rungStrat();

        // do we have time to get back to the user zone?

        // Make sure robot stops and claw is open (teleop initialization default) before OpMode dies
        robot.stop();
        robot.openClaw(false);

    }
}
