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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Concept: Sgp Wrist Servos", group = "Concept")
public class Sgp_Concept_WristServos extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double[] RANGE_FULL= {0.0, 1.0};
    static final double[] RANGE_TOP_HALF = {0.5, 1.0};
    static final double[] RANGE_BOTTOM_HALF = {0.0, 0.5};

    // Define class members
    Servo   Wrist_1;
    Servo   Wrist_2;
    Servo   Claw;
    Servo   activeServo = null;
    ServoController activeServoController = null;
    boolean rampUp = true;
    double[] range = RANGE_FULL;
    double  position = (range[1] - range[0]) / 2; // Start at halfway position
    double increment = INCREMENT;

    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        Wrist_1 = hardwareMap.get(Servo.class, "Wrist_1" );
        Wrist_2 = hardwareMap.get(Servo.class, "Wrist_2");
        Claw = hardwareMap.get(Servo.class, "Finger");
        boolean buttonPressed = false;

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();

        Wrist_1.setPosition((RANGE_FULL[1] - RANGE_FULL[0])/2);
        Wrist_2.setPosition((RANGE_FULL[1] - RANGE_FULL[0])/2);
        Claw.setPosition ((RANGE_FULL[1] - RANGE_FULL[0])/2);
        activeServo = null;
        activeServoController = null;

        // Scan servo till stop pressed.
        while(opModeIsActive()){

            // If trigger is pulled, ramp down, otherwise ramp up.
            rampUp = gamepad2.right_trigger == 0f;

            // If gamepad2 X, then activate Wrist_1
            if( gamepad2.x)
            {
                activeServo = Wrist_1;
                range = RANGE_FULL;
            }
            // If gamepad2 Y, then activate Wrist_2
            else if ( gamepad2.y)
            {
                activeServo = Wrist_2;
                range = RANGE_FULL;
            }
            // If gamepad2 Y, then activate Claw
            else if (gamepad2.b)
            {
                activeServo = Claw;
                range = RANGE_FULL;
                // If switching to claw with an initial position less than where the claw can go,
                // reset it to start at the min of range. Expect that claw may jump to this state
                // if its current position is anything but closed.
//                if(position < range[0] )
//                    position = range[0];
            }
            else if (gamepad2.a)
            {
                // Reset
                range = RANGE_FULL;
                position = (range[1] - range[0])/2;
                Wrist_1.setPosition((range[1] - range[0])/2);
                Wrist_2.setPosition((range[1] - range[0])/2);
                Claw.setPosition ((range[1] - range[0])/2);
                activeServo = null;
                activeServoController = null;
            }
            else
            {
                activeServo = null;
                activeServoController = null;
            }
            if(activeServo == null)
                continue;

            // slew the servo, according to the rampUp (direction) variable.
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                position += increment ;
                if (position >= range[1] ) {
                    position = range[1];
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                position -= increment ;
                if (position <= range[0] ) {
                    position = range[0];
                }
            }

            // Set the servo to the new position and pause;
            activeServo.setPosition(position);

            telemetry.addData("Active Servo Port", activeServo.getPortNumber());
            telemetry.addData( "Get Position","%5.2f",activeServo.getPosition() );
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
