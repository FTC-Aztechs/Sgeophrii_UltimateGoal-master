/*
Copyright 2020 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class Motors extends LinearOpMode {
    private Blinker expansion_Hub_2;
    private DcMotor left_Motor;
    private DcMotor right_Motor;
    private Gyroscope imu;


    @Override
    public void runOpMode() {
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        left_Motor = hardwareMap.get(DcMotor.class, "Left_Motor");
        right_Motor = hardwareMap.get(DcMotor.class, "Right_Motor");
        imu = hardwareMap.get(Gyroscope.class, "imu");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        //variables for motors
        //left motor
        double tgtPower1 = 0.15;
        //right motor
        double tgtPower2 = 0;
        //speed variable
        double speed = 2;
        //speed limit
        double speedLimit = 2.3;
        while (opModeIsActive()) {
            //put code here
            if(gamepad1.y){
                //add to speed if less than or equal to speed limit
                if (speed < speedLimit + 1){
                    speed++;
                };
            }else if(gamepad1.a){
                // decrease speed if more than or equal to negative speed limit)
                if (speed > (speedLimit*-1)-1){
                    speed--;
                };
            }else if(gamepad1.x){
                //set speed to 0
                speed = 0;
            }else if(gamepad1.b){
                //set speed to one
                speed = 1;
            }
            //set the tgtPower to gamePad stick output(-1/1) * speed
            tgtPower2 = this.gamepad1.left_stick_y * speed;
            tgtPower1 = -this.gamepad1.right_stick_y * speed;
            //set motor power
            left_Motor.setPower(tgtPower1);
            right_Motor.setPower(tgtPower2);

            //add and update variables
            telemetry.addData("Target Power", "left: " + tgtPower1 + ", right: " + tgtPower2);
            telemetry.addData("Motor Power", "left: " + left_Motor.getPower() + ", right: " + right_Motor.getPower());
            telemetry.addData("Status", "Running");
            telemetry.addData("Speed", speed);
            telemetry.update();
        }
    }
}