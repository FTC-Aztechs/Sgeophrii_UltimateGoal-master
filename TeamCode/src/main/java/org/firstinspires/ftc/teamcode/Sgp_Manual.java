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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Sgp_Manual", group="Manual mode")
// @Disabled
public class Sgp_Manual extends LinearOpMode
{
    // Declare OpMode members.
    SgpRobot robot = new SgpRobot();

    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.779;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;
    static final double RANGE[] = {0.0, 1.0};
    static final int CYCLE_MS = 50;
    static final double INCREMENT = 0.25;

    private boolean rampUp = true;
    private double speedAdjust = 7;
    private ElapsedTime runtime = new ElapsedTime();
    private double position = (RANGE[1] - RANGE[0]) / 2;

    @Override
    public void runOpMode() {

        // Initialize the drive system vriables
        robot.init(hardwareMap);
        telemetry.addData("Status", "Init Hardware");
        telemetry.update();

        robot.setRunMode(SgpRobot.SgpMotors.ALL, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(SgpRobot.SgpMotors.ALL, DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        initSgeophrii();

        while(opModeIsActive())
        {
            driveSgeophrii();
            armSgeophrii();
            shootSgeophrii();
        }
    }

    public void initSgeophrii() {
        // set Arm positions
        position = RANGE[1] - RANGE[0] / 2;
        robot.Wrist_1.setPosition(position);
        robot.Wrist_2.setPosition(position);

        telemetry.addData("Status:", "Sgeophrii initialized");
        telemetry.update();

        return;
    }

    public void driveSgeophrii()
    {
        if( gamepad1.dpad_left && speedAdjust >= 1 ) {
            speedAdjust -= 1;
            telemetry.addData("Current speed: ", "%d", speedAdjust);
            telemetry.update();
        }

        if(gamepad1.dpad_right && speedAdjust <= 7) {
            speedAdjust += 1;
            telemetry.addData("Current speed: ", "%d", speedAdjust);
            telemetry.update();
        }

        // Set Motor pover
        robot.lower_left.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1. right_stick_x)*(-speedAdjust/10));
        robot.lower_right.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1. right_stick_x)*(-speedAdjust/10));
        robot.upper_left.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1. right_stick_x)*(-speedAdjust/10));
        robot.upper_right.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1. right_stick_x)*(-speedAdjust/10));

        return;
    }

    public void armSgeophrii()
    {
        Servo activeServo = null;
        rampUp = gamepad2.right_trigger == 0f;

        if( gamepad2.x)
            activeServo = robot.Wrist_1;
        else if (gamepad2.y)
            activeServo = robot.Wrist_2;
        else if( gamepad2.b)
            activeServo = robot.Finger;
        else if (gamepad2.a)
        {
            initSgeophrii();
            activeServo = null;
        }
        else
        {
            activeServo = null;
        }
        if( activeServo == null)
            return;

        if( rampUp) {
            position += INCREMENT;
            if (position >= RANGE[1]) {
                position = RANGE[1];
            }
        }
        else {
            position -= INCREMENT;
            if (position <= RANGE[0]) {
                position = RANGE[0];
            }
        }

        activeServo.setPosition(position);
        String activeServoName;
        int servoPort = activeServo.getPortNumber();
        if( servoPort == 0)
            activeServoName = "Wrist_1";
        else if( servoPort == 1)
            activeServoName = "Wrist_2";
        else
            activeServoName = "Finger";
        telemetry.addData("Active Servo: ", activeServoName);
        telemetry.addData("Servo Position: ", activeServo.getPosition());
        telemetry.update();

        return;
    }

    public void shootSgeophrii()
    {
        return;
    }
}
