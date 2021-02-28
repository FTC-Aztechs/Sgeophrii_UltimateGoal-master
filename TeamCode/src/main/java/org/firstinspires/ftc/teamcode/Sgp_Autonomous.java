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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * ToDo: [Lavanya/Atiksh]
 *      1. Document what this class does
 *      2. Add a if(opModeIsActive()) check to function implementations *
 **/

@Autonomous(name="Sgp_Autonomous", group="Autonomous Mode")
public class Sgp_Autonomous extends LinearOpMode {

    /* Declare OpMode members. */
    SgpRobot robot = new SgpRobot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.779;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 0.9;

    OpenCvInternalCamera    phoneCam;
    SgpDeterminationPipeline pipeline;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //robot.Arm_Motor.setPower(0.1);
        telemetry.setAutoClear(false);
        telemetry.addData("Status", "Init Hardware");
        telemetry.update();

        robot.setRunMode(SgpRobot.SgpMotors.ALL, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(SgpRobot.SgpMotors.ALL, DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status: ", "Initializing camera......");
        telemetry.update();

        acquireCamera();
        SgpDeterminationPipeline.RingPosition Position = pipeline.position;
        telemetry.addData("Status: ", "Ready");
        telemetry.update();

        waitForStart();

        Position = pipeline.position;
        telemetry.addData("Analysis", pipeline.getAnalysis());
        telemetry.addData("Position", Position);
        telemetry.update();

        runtime.reset();
        robot.Finger.setPosition(0.0);

        telemetry.setAutoClear(true);

        // Shoot preloaded rings
        sgpAutoShoot();

        switch (Position) {
            case NONE:
                // Start from initial position, go to drop zone A,
                // drop the wobble goal, trace back to launch zone
                WobbleDropPositionA();
                break;
            case ONE:
                WobbleDropPositionB();
                break;
            case FOUR:
                WobbleDropPositionC();
                break;
        }

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }


    public void WobbleDropPositionA()
    {
        if(opModeIsActive()) {

            // Go to Position A
            sgpAutoDrive(DRIVE_SPEED, -5, -5, 30);
            sgpAutoStrafe(DRIVE_SPEED, 22, 22, 30.0);
            sgpAutoDrive(DRIVE_SPEED, 24,24,  30.0);  // S3: Forward 13 Inches with 4 Sec timeout

            dropWobbleOnField();

            sgpAutoStrafe(DRIVE_SPEED, -22, -22, 30.0);
            sgpAutoDrive(DRIVE_SPEED, 14, 14, 30.0);

            telemetry.addData("Status", "Reached Position A");
            telemetry.update();
        }
        return;
    }

    public void WobbleDropPositionB() {
        if(opModeIsActive()) {

            //sgpAutoStrafe(DRIVE_SPEED, 4, 4, 30.0);
            sgpAutoDrive(DRIVE_SPEED, 42, 42, 30.0);  // S3: Forward 13 Inches with 4 Sec timeout
            dropWobbleOnField();

            telemetry.addData("Status", "Reached Position B");
            telemetry.update();
        }

        return;
    }

    public void WobbleDropPositionC() {
        if(opModeIsActive()) {
            sgpAutoDrive(DRIVE_SPEED, -5, -5, 30);
            sgpAutoStrafe(DRIVE_SPEED, 22, 22, 30.0);
            sgpAutoDrive(DRIVE_SPEED, 68, 68, 30.0);  // S3: Forw
            dropWobbleOnField();
            sgpAutoDrive(DRIVE_SPEED, -31, -31, 30.0);

            telemetry.addData("Status", "Reached Position C");
            telemetry.update();
        }

        return;
    }

    public void dropWobbleOnField()
    {
        if(opModeIsActive()) {

            sgpAutoDrive(DRIVE_SPEED,-6, 6, 30);
            robot.setRunMode(SgpRobot.SgpMotors.ARM_MOTOR, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setRunMode(SgpRobot.SgpMotors.ARM_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Arm Motor position0:", robot.getCurrentPosition(SgpRobot.SgpMotors.ARM_MOTOR));
//
            robot.setTargetPosition(SgpRobot.SgpMotors.ARM_MOTOR, 400);
            robot.setPower(SgpRobot.SgpMotors.ARM_MOTOR, 1);
            robot.setRunMode(SgpRobot.SgpMotors.ARM_MOTOR, DcMotor.RunMode.RUN_TO_POSITION);
            while(opModeIsActive() && robot.areMotorsBusy(SgpRobot.SgpMotors.ARM_MOTOR)) {
                telemetry.addData("Status: ","Arm moving forward");
                telemetry.update();
            }


            robot.setTargetPosition(SgpRobot.SgpMotors.ARM_MOTOR, 700);
            robot.setPower(SgpRobot.SgpMotors.ARM_MOTOR, 0.4);
            robot.setRunMode(SgpRobot.SgpMotors.ARM_MOTOR, DcMotor.RunMode.RUN_TO_POSITION);
            while(opModeIsActive() && robot.areMotorsBusy(SgpRobot.SgpMotors.ARM_MOTOR)) {
                telemetry.addData("Status: ","Arm moving forward");
                telemetry.update();
            }


            sleep(1000);
            robot.Finger.setPosition(1);
            sleep(1000);
            robot.Finger.setPosition(0);

            telemetry.addData("Arm Motor position1:", robot.getCurrentPosition(SgpRobot.SgpMotors.ARM_MOTOR));
            robot.setTargetPosition(SgpRobot.SgpMotors.ARM_MOTOR, 0);
            robot.setPower(SgpRobot.SgpMotors.ARM_MOTOR, 1);
            while(opModeIsActive() && robot.areMotorsBusy(SgpRobot.SgpMotors.ARM_MOTOR)) {
                telemetry.addData("Status: ","Arm moving backward");
                telemetry.update();
            }
            robot.setPower(SgpRobot.SgpMotors.ARM_MOTOR,0);

            telemetry.addData("Arm Motor position2:", robot.getCurrentPosition(SgpRobot.SgpMotors.ARM_MOTOR));
            telemetry.addData("Status", "Dropping Wobble goal on field");
            telemetry.update();
            sgpAutoDrive(DRIVE_SPEED,6,-6, 30);
        }

        return;
    }

    public void PickupWobble2()
    {
        if(opModeIsActive()) {
            telemetry.addData("Status" , "Picked up Wobble 2");
            telemetry.update();
        }

        return;
    }

    public void sgpAutoShoot()
    {
        if(opModeIsActive()) {
            sgpAutoDrive(1, 7, 7, 30);
            sgpAutoStrafe(1, -8, -8, 30);

            sgpAutoDrive(1, 26,26,30);
            robot.Bravo_6.setPower(0.95);
            sleep(3000);
           for (int shootingTimes = 0; shootingTimes< 4; shootingTimes++) {
               robot.Shooter_Servo.setPosition(robot.Pusher_Pos + 0.25);
               sleep(500);
               robot.Shooter_Servo.setPosition(robot.Pusher_Pos);
               sleep(500);
           }
           robot.Bravo_6.setPower(0);
           telemetry.addData("Status", "Shooting Rings Complete");
           telemetry.update();
        }

        return;
    }

    public void navigateToLaunchLine()
    {
        if(opModeIsActive()) {

            telemetry.addData("Status", "Navigation to launch line completed");
            telemetry.update();
        }

        return;
    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *  4) Any of the motors stops
     */
    public void sgpAutoDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS)
    {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Reset
            robot.setRunMode(SgpRobot.SgpMotors.ALL, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setRunMode(SgpRobot.SgpMotors.ALL, DcMotor.RunMode.RUN_USING_ENCODER);

            // Determine new target position, and pass to motor controller
            newRightTarget = (int)(leftInches  * COUNTS_PER_INCH);
            newLeftTarget  = (int)(rightInches * COUNTS_PER_INCH);

            robot.setTargetPosition(SgpRobot.SgpMotors.UPPER_LEFT, newLeftTarget);
            robot.setTargetPosition(SgpRobot.SgpMotors.LOWER_RIGHT, newRightTarget);
            robot.setTargetPosition(SgpRobot.SgpMotors.UPPER_RIGHT, newRightTarget);
            robot.setTargetPosition(SgpRobot.SgpMotors.LOWER_LEFT, newLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.setRunMode(SgpRobot.SgpMotors.ALL_DRIVES, DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.setPower(SgpRobot.SgpMotors.ALL_DRIVES, Math.abs(speed));

            while (opModeIsActive() &&
                  (runtime.seconds() < timeoutS) &&
                  (robot.areMotorsBusy(SgpRobot.SgpMotors.ALL_DRIVES)) )
            {
                telemetry.addData("Target positions", "Left %7d : Right %7d", newLeftTarget, newRightTarget);
                telemetry.addData("Current  position", "Upper_Left %7d : Lower_Right %7d : Upper_Right %7d : Lower_Left %7d",
                        robot.getCurrentPosition(SgpRobot.SgpMotors.UPPER_LEFT),
                        robot.getCurrentPosition(SgpRobot.SgpMotors.LOWER_RIGHT),
                        robot.getCurrentPosition(SgpRobot.SgpMotors.UPPER_RIGHT),
                        robot.getCurrentPosition(SgpRobot.SgpMotors.LOWER_LEFT));
                telemetry.update();
            }

            // Stop all motion;
            robot.setPower(SgpRobot.SgpMotors.ALL_DRIVES, 0);

            // Turn off RUN_TO_POSITION
            robot.setRunMode(SgpRobot.SgpMotors.ALL_DRIVES, DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    public void sgpAutoStrafe(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        robot.setRunMode(SgpRobot.SgpMotors.ALL, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(SgpRobot.SgpMotors.ALL, DcMotor.RunMode.RUN_USING_ENCODER);

        if (opModeIsActive()) {
            newLeftTarget = robot.getCurrentPosition(SgpRobot.SgpMotors.LOWER_LEFT) + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.getCurrentPosition(SgpRobot.SgpMotors.LOWER_RIGHT) + (int) (rightInches * COUNTS_PER_INCH);

                 //    Strafe Right ->      Strafe Left <-
                 //    ^^  O-----O vv       vv  O-----O ^^
                 //           |                    |
                 //           |                    |
                 //    vv  O-----O ^^       ^^  O-----O vv

            robot.setTargetPosition(SgpRobot.SgpMotors.LOWER_LEFT, newLeftTarget);
            robot.setTargetPosition(SgpRobot.SgpMotors.UPPER_RIGHT, newRightTarget);

            robot.setTargetPosition(SgpRobot.SgpMotors.LOWER_RIGHT, -newRightTarget);
            robot.setTargetPosition(SgpRobot.SgpMotors.UPPER_LEFT, -newLeftTarget);


            runtime.reset();            robot.setRunMode(SgpRobot.SgpMotors.ALL_DRIVES, DcMotor.RunMode.RUN_TO_POSITION);

            robot.setPower(SgpRobot.SgpMotors.ALL_DRIVES, Math.abs(speed));

            while (opModeIsActive()
                    && runtime.seconds() < timeoutS
                    && robot.areMotorsBusy(SgpRobot.SgpMotors.ALL_DRIVES)) {
                telemetry.addData("Target positions", "Left %7d : Right %7d", newLeftTarget, newRightTarget);
                telemetry.addData("Current position", "Upper_Left %7d : Lower_Right %7d : Upper_Right %7d : Lower_Left %7d",
                        robot.getCurrentPosition(SgpRobot.SgpMotors.UPPER_LEFT),
                        robot.getCurrentPosition(SgpRobot.SgpMotors.LOWER_RIGHT),
                        robot.getCurrentPosition(SgpRobot.SgpMotors.UPPER_RIGHT),
                        robot.getCurrentPosition(SgpRobot.SgpMotors.LOWER_LEFT));
                telemetry.update();
            }

            // Stop all motion
            robot.setPower(SgpRobot.SgpMotors.ALL_DRIVES, 0);
            robot.setRunMode(SgpRobot.SgpMotors.ALL_DRIVES, DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void acquireCamera()
    {
                                                                                                                                                                                                                                                                                                                                                                                                                                                       int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        pipeline = new SgpDeterminationPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });

       return ;
    }

    /**
     * ToDo: 1. Avi/Dhruv - please document what this class does
     *       2. Investigate how this class can be moved into a new file
     *
    */
    public static class SgpDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
                */
        //static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(121000,60);
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(60,50);


        static final int REGION_WIDTH = 45;
        static final int REGION_HEIGHT = 35;

        final int FOUR_RING_THRESHOLD = 160;
        final int ONE_RING_THRESHOLD = 139;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile SgpDeterminationPipeline.RingPosition position = SgpDeterminationPipeline.RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = SgpDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = SgpDeterminationPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = SgpDeterminationPipeline.RingPosition.ONE;
            }else{
                position = SgpDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}


