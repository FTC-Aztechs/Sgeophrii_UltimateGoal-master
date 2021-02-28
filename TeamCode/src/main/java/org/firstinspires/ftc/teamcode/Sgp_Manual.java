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
public class Sgp_Manual extends LinearOpMode {
    // Declare OpMode members.
    SgpRobot robot = new SgpRobot();

    static final double SERVO_POSITION = 0.5;
    static final double RANGE[] = {0.0, 1.0};
    static final int CYCLE_MS = 50;
    static final double INCREMENT = 0.25;

    private boolean rampUp = true;
    private double speedAdjust = 10;
    private ElapsedTime runtime = new ElapsedTime();
    private double position = (RANGE[1] - RANGE[0]) / 2;
    private double Wrist_pos = (RANGE[1] - RANGE[0]) / 2;
    private double Finger_pos = RANGE[0];
    double Arm_Power = 0;
    double maxArm_Power = 0.5;
    double shooter_power = 0.1;
    double intake_power = 0.9;
    double Pusher_Pos = 0;
    boolean shooterOn = false;
    boolean intakeOn = false;

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

        telemetry.setAutoClear(false);

        while (opModeIsActive()) {
            sgpManualDrive();
            sgpManualArm();
            sgpManualShoot();
            sgpManualIntake();
        }
    }


    public void initSgeophrii() {
        // set Arm positions
        position = RANGE[1] - RANGE[0] / 2;
        // robot.Shooter_Servo.setPosition(position);
        Pusher_Pos = robot.Shooter_Servo.getPosition();
        robot.Wrist_2.setPosition(Wrist_pos);
        robot.Finger.setPosition(Finger_pos);

        telemetry.addData("Pusher Position", Pusher_Pos);
        telemetry.addData("Status:", "Sgeophrii initialized");
        telemetry.update();

        return;
    }

    public void sgpManualDrive() {
        if (gamepad1.dpad_left && speedAdjust >= 1) {
            speedAdjust -= 1;
            telemetry.addData("Current speed: ", "%f", speedAdjust);
            telemetry.update();
        }

        if (gamepad1.dpad_right && speedAdjust <= 7) {
            speedAdjust += 1;
            telemetry.addData("Current speed: ", "%f", speedAdjust);
            telemetry.update();
        }

        // TO DO : Fix the "Strafe" portion of this code.
        //        What we learnt today is that the lef_stick_x seems to make things go bad.

        // Set Motor power Original
//        robot.lower_left.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1. right_stick_x)*(-speedAdjust/10)); // 1.0
//        robot.lower_right.setPower((gamepad1.left_stick_y  gamepad1.left_stick_x + gamepad1. right_stick_x)*(-speedAdjust/10)); // 1.0
//        robot.upper_left.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1. right_stick_x)*(-speedAdjust/10)); // 0
//        robot.upper_right.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1. right_stick_x)*(-speedAdjust/10)); // 0

        // Set Motor power Lavanya 1
        // See Mecanum_Drive spreadsheet
//        robot.lower_left.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1. right_stick_x)*(-speedAdjust/10)); // 1.0
//        robot.lower_right.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1. right_stick_x)*(-speedAdjust/10)); // 1.0
//        robot.upper_left.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1. right_stick_x)*(-speedAdjust/10)); // 0
//        robot.upper_right.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1. right_stick_x)*(-speedAdjust/10)); // 0

        //Lavanya 3 (the wonder algorithm!!!)
        robot.lower_left.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10)); // 1.0
        robot.lower_right.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10)); // 1.0
        robot.upper_left.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10)); // 0
        robot.upper_right.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10)); // 0

        return;
    }

    public void sgpManualArm() {
        Servo activeServo = null;
        rampUp = gamepad2.right_trigger == 0f;

        if (gamepad2.b) {
            if (rampUp) {
                Finger_pos += INCREMENT;
                if (Finger_pos >= RANGE[1])
                    Finger_pos = RANGE[1];
            } else {
                Finger_pos -= INCREMENT;
                if (Finger_pos <= RANGE[0])
                    Finger_pos = RANGE[0];
            }
            robot.Finger.setPosition(Finger_pos);
        } else if (gamepad2.a) {
            initSgeophrii();
            activeServo = null;
        } else {
            activeServo = null;
        }


        String activeServoName;
        double servoPosition = 0.0f;

        //set the arm motor's power
        Arm_Power = gamepad2.right_stick_y * maxArm_Power;
        robot.Arm_Motor.setPower(Arm_Power);

        return;
    }

    public void sgpManualShoot() {
        //Shooter and pusher code
        if (gamepad2.x) {
            telemetry.addData("Shooter: ", shooterOn);
            shooterOn = !shooterOn;
            telemetry.addData("Shooter Toggle to:", shooterOn);
            if (shooterOn) {
                telemetry.addData("Turned on power at: ", "%f", shooter_power);
            }
            sleep(500);
        } else if (gamepad2.dpad_down) {
            shooter_power -= 0.1;
            if (shooter_power < 0.1) {
                shooter_power = 0.1;
            }
            sleep(500);
            telemetry.addData("shooter_power: ", "%f", shooter_power);
        } else if (gamepad2.dpad_up) {
            shooter_power += 0.1;
            if (shooter_power > 1.0) {
                shooter_power = 1.0;
            }
            sleep(500);
            telemetry.addData("shooter_power: ", "%f", shooter_power);
        }

        if (shooterOn) {
            robot.Bravo_6.setPower(shooter_power);
        } else {
            robot.Bravo_6.setPower(0);
        }

        if (gamepad2.left_trigger == 1f) {
            telemetry.addData("pusher position", robot.Shooter_Servo.getPosition());
            robot.Shooter_Servo.setPosition(Pusher_Pos + 0.25);
            telemetry.addData("pusher position", robot.Shooter_Servo.getPosition());
            sleep(500);
            robot.Shooter_Servo.setPosition(Pusher_Pos);
            telemetry.addData("Pusher Servo", "Push Complete");
        }

        telemetry.update();
        return;
    }

    public void sgpManualIntake() {

        if (gamepad2.y) {
            telemetry.addData("Intake: ", intakeOn);
            intakeOn = !intakeOn;
            telemetry.addData("Intake Toggle to: ", intakeOn);
            if (intakeOn) {
                telemetry.addData("Turned on power at: ", "%f", intake_power);
            }
            sleep(500);
        } else if (gamepad2.dpad_left) {
            intake_power -= 0.1;
            if (intake_power < 0.1) {
                intake_power = 0.1;
            }
            sleep(500);
            telemetry.addData("intake_power: ", "%f", intake_power);
        } else if (gamepad2.dpad_right) {
            intake_power += 0.1;
            if (intake_power > 1.0)
                intake_power = 1.0;
            sleep(500);
            telemetry.addData("intake_power: ", "%f", intake_power);
        }

        if (intakeOn) {
            robot.setPower(SgpRobot.SgpMotors.INTAKE, -intake_power);
        } else {
            robot.setPower(SgpRobot.SgpMotors.INTAKE, 0);
        }

        telemetry.update();
        return;
    }
}