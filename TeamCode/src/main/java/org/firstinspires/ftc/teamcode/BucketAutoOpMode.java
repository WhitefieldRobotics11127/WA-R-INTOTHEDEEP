/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Left-side (Bucket) Autonomous", group="Competition")
//@Disabled


public class BucketAutoOpMode extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." to access this class.
    RobotHardware robot = new RobotHardware(this);

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    // 0.5 = straight up
    public void dropInBucket() {
        //rotate arm upwards to field.
        if (opModeIsActive())
            robot.setArmRotation(0.14);
        //extend arm
        if (opModeIsActive())
            robot.setArmExtension(1650);
        //approach bucket
        if (opModeIsActive())
            robot.forward(90, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);
        //release claw to drop block and close claw
        if (opModeIsActive())
            robot.openClaw(true);
        //extend arm
        if (opModeIsActive())
            robot.setArmExtension(1650);
        //the Scoot!
        if (opModeIsActive())
            robot.forward(-90, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS );
        //retract the arm
        if (opModeIsActive())
            robot.setArmExtension(robot.ARM_EXTENSION_MIN);
        //rotate arm back
        if (opModeIsActive())
            robot.setArmRotation(RobotHardware.ARM_ROTATION_MIN);
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
            robot.forward(970,RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

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
        if(opModeIsActive())
            robot.turn(-2.21, RobotHardware.MOTOR_SPEED_FACTOR_AUTONOMOUS);

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
            robot.setArmRotation(0.24);

    }
}
