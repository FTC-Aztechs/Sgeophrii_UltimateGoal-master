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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * ToDo: Vishana/Anshul - please document what this class does
 */


@TeleOp(name="Sgp_Manual", group="Manual mode")
// @Disabled
public class Sgp_Manual extends LinearOpMode
{
    // Declare OpMode members.
    SgpRobot robot = new SgpRobot();

    static final double SERVO_POSITION = 0.5;
    static final double RANGE[] = {0.0, 1.0};
    static final int CYCLE_MS = 50;
    static final double INCREMENT = 0.25;

    private boolean rampUp = true;
    private double speedAdjust = 7;
    private ElapsedTime runtime = new ElapsedTime();
    private double position = (RANGE[1] - RANGE[0]) / 2;
    double Arm_Power = 0;
    double maxArm_Power=  0.5;

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
            sgpManualDrive();
            sgpManualArm();
            sgpManualShoot();
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

    public void sgpManualDrive()
    {
        if( gamepad1.dpad_left && speedAdjust >= 1 ) {
            speedAdjust -= 1;
            telemetry.addData("Current speed: ", "%f", speedAdjust);
            telemetry.update();
        }

        if(gamepad1.dpad_right && speedAdjust <= 7) {
            speedAdjust += 1;
            telemetry.addData("Current speed: ", "%f", speedAdjust);
            telemetry.update();
        }

        // TO DO : Fix the "Strafe" portion of this code.
        //        What we learnt today is that the lef_stick_x seems to make things go bad.
        // left_stick_y = 1.0
        // left_stick_x = -0.5
        // right_stick_x = 0

        // Set Motor power Original
//        robot.lower_left.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1. right_stick_x)*(-speedAdjust/10)); // 1.0
//        robot.lower_right.setPower((gamepad1.left_stick_y  gamepad1.left_stick_x + gamepad1. right_stick_x)*(-speedAdjust/10)); // 1.0
//        robot.upper_left.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1. right_stick_x)*(-speedAdjust/10)); // 0
//        robot.upper_right.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1. right_stick_x)*(-speedAdjust/10)); // 0

        // Set Motor power Lavanya
        robot.lower_left.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1. right_stick_x)*(-speedAdjust/10)); // 1.0
        robot.lower_right.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1. right_stick_x)*(-speedAdjust/10)); // 1.0
        robot.upper_left.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1. right_stick_x)*(-speedAdjust/10)); // 0
        robot.upper_right.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1. right_stick_x)*(-speedAdjust/10)); // 0


        //gamepad1.left_stick_y: Forward/Backward
        //gamepad1.left_stick_x: Turn left / Right
        //gamepad1.left_stick_x + LT: Strafe left / right

          return;
    }

    public void sgpManualArm()
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

        String activeServoName;
        double servoPosition=0.0f;

        if(activeServo !=null) {
            activeServo.setPosition(position);
            int servoPort = activeServo.getPortNumber();
            servoPosition = activeServo.getPosition();
            if (servoPort == 0)
                activeServoName = "Wrist_1";
            else if (servoPort == 1)
                activeServoName = "Wrist_2";
            else
                activeServoName = "Finger";
        }
        else {
            activeServoName = "none";
        }
        //set the arm motor's power

        Arm_Power = gamepad2.right_stick_y * maxArm_Power;
        robot.Arm_Motor.setPower(Arm_Power);

        telemetry.addData("Active Servo: ", activeServoName);
        telemetry.addData("Active Servo set position", position);
        telemetry.addData("Servo Position: ", servoPosition);
        telemetry.addData("Arm Motor Power", Arm_Power);
        telemetry.update();

        return;
    }

    public void sgpManualShoot() {
        if (gamepad2.dpad_left && speedAdjust >= 1) {
            speedAdjust -= 1;
            telemetry.addData("Current speed: ", "%f", speedAdjust);
            telemetry.update();
        }

        if (gamepad2.dpad_right && speedAdjust <= 7) {
            speedAdjust += 1;
            telemetry.addData("Current speed: ", "%f", speedAdjust);
            telemetry.update();
        }

        robot.Batman_Belt.setPower((gamepad2.right_stick_y) * (-speedAdjust / 10));
        robot.Bravo_6.setPower((gamepad2.left_stick_y) * (-speedAdjust / 10));

        return;
    }
}
