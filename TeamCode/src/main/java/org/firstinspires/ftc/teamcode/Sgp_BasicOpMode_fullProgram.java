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

@TeleOp(name="Basic: Iterative fullProgram", group="Iterative Opmode")
// @Disabled
public class Sgp_BasicOpMode_fullProgram extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor upper_right = null;
    private DcMotor upper_left = null;
    private DcMotor lower_left = null;
    private DcMotor lower_right = null;
    private DcMotor Arm_Motor = null;
    private Servo Wrist_1 = null;
    private Servo Wrist_2 = null;
    private Servo Finger = null;




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        upper_right  = hardwareMap.get(DcMotor.class, "upper_right");
        upper_left = hardwareMap.get(DcMotor.class, "upper_left");
        lower_left = hardwareMap.get(DcMotor.class, "lower_left");
        lower_right = hardwareMap.get(DcMotor.class, "lower_right");
        Arm_Motor = hardwareMap.get(DcMotor.class, "Arm_Motor");
        Wrist_1 = hardwareMap.get(Servo.class, "Wrist_1");
        Wrist_2 = hardwareMap.get(Servo.class, "Wrist_2");
        Finger = hardwareMap.get(Servo.class, "Finger");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        upper_left.setDirection(DcMotor.Direction.FORWARD);
        upper_right.setDirection(DcMotor.Direction.REVERSE);
        lower_left.setDirection(DcMotor.Direction.FORWARD);
        lower_right.setDirection(DcMotor.Direction.REVERSE);
        Arm_Motor.setDirection(DcMotor.Direction.FORWARD);
        Wrist_1.setDirection(Servo.Direction.FORWARD );
        Wrist_2.setDirection(Servo.Direction.FORWARD );
        Finger.setDirection(Servo.Direction.FORWARD );

        //zero power behavior
        upper_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }
    double speedAdjust = 7;
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.dpad_left == true) {
            speedAdjust -= 1;
        }
        if(gamepad1.dpad_right == true) {
            speedAdjust += 1;
        }
        //set motor power
        lower_left.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1. right_stick_x)*(-speedAdjust/10));
        lower_right.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1. right_stick_x)*(-speedAdjust/10));
        upper_left.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1. right_stick_x)*(-speedAdjust/10));
        upper_right.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1. right_stick_x)*(-speedAdjust/10));

        //set arm's power
        Arm_Motor.setPower(gamepad2.left_stick_y );
        Wrist_1.setPosition(gamepad2.right_stick_y);
        Wrist_2.setPosition(gamepad2.left_stick_x);
        Finger.setPosition(gamepad2.right_stick_x);




        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
       telemetry.addData("Speed Adjust", speedAdjust) ;
       telemetry.addData( "Arm Data" , "Arm Motor: " + Arm_Motor.getCurrentPosition() + ", Wrist 1 Servo: " + Wrist_1.getPosition() + ", Wrist 2 Servo: " + Wrist_2.getPosition() + ", Finger Servo: " + Finger.getPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
