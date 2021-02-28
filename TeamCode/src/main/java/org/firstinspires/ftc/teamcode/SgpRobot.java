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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Locale;

 /**
 * ToDo: Vishruth/Satvika - please document what this class does
 */

public class SgpRobot
{
    enum SgpMotors
    {
        UPPER_LEFT,
        LOWER_LEFT,
        UPPER_RIGHT,
        LOWER_RIGHT,
        ARM_MOTOR,
        INTAKE,
        BRAVO6,
        BATMAN_BELT,
        ALL_DRIVES,
        ALL
    }
    /* Public OpMode members. */
    public DcMotor upper_right = null;
    public DcMotor upper_left = null;
    public DcMotor lower_left = null;
    public DcMotor lower_right = null;
    public DcMotor Arm_Motor = null;
    public DcMotor Intake = null;
    public DcMotor Batman_Belt = null;
    public DcMotor Bravo_6 = null;
    public Servo Shooter_Servo = null;
    public Servo Wrist_2 = null;
    public Servo Finger = null;
    public ServoController FingerController = null;
    public BNO055IMU imu_gyro = null;
    OpenCvInternalCamera phoneCam;
    EasyOpenCVExample.SkystoneDeterminationPipeline pipeline;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    double Pusher_Pos = 0;
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public SgpRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        upper_right  = hwMap.get(DcMotor.class, "upper_right");
        upper_left = hwMap.get(DcMotor.class, "upper_left");
        lower_left = hwMap.get(DcMotor.class, "lower_left");
        lower_right = hwMap.get(DcMotor.class, "lower_right");
        Arm_Motor = hwMap.get(DcMotor.class, "Arm_Motor");
        Intake = hwMap.get(DcMotor.class, "Intake");
        Batman_Belt = hwMap.get(DcMotor.class, "Batman_Belt");
        Bravo_6 = hwMap.get(DcMotor.class, "Bravo_6");
        Shooter_Servo = hwMap.get(Servo.class, "Wrist_1");
        Wrist_2 = hwMap.get(Servo.class, "Wrist_2");
        Finger = hwMap.get(Servo.class, "Finger");

        // Acquire gyro
        initImuGyro();


        // Set all motors to zero power
        upper_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Arm_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Batman_Belt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Bravo_6.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        upper_left.setDirection(DcMotor.Direction.FORWARD);  //-
        upper_right.setDirection(DcMotor.Direction.REVERSE); //+

        //lower_left.setDirection(DcMotor.Direction.REVERSE); //-
        lower_left.setDirection(DcMotor.Direction.FORWARD); //- used to be

        lower_right.setDirection(DcMotor.Direction.REVERSE); //+ used to be
        //lower_right.setDirection(DcMotor.Direction.FORWARD); //+

        Arm_Motor.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Batman_Belt.setDirection(DcMotor.Direction.FORWARD);
        Bravo_6.setDirection(DcMotor.Direction.FORWARD);
        Shooter_Servo.setDirection(Servo.Direction.FORWARD );
        Wrist_2.setDirection(Servo.Direction.FORWARD );
        Finger.setDirection(Servo.Direction.FORWARD );

        //Shooter_Servo.setPosition(0);
        Wrist_2.setPosition(0.5);
        Finger.setPosition (0.0);
        Pusher_Pos = Shooter_Servo.getPosition();
    }

    public void initImuGyro()
    {
        if(imu_gyro == null)
        {
            imu_gyro = hwMap.get(BNO055IMU.class, "imu");
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu_gyro.initialize(parameters);
    }

//    public void imuTelemetry(Telemetry telemetry)
//    {
//        if(imu_gyro == null)
//            initImuGyro();
//        final Orientation angles;
//        final Acceleration gravity;
//
//        telemetry.addAction( new Runnable() {
//                                 @Override
//                                 public void run() {
//                                     angles = imu_gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                                     gravity = imu_gyro.getGravity();
//                                 }
//                             });
//
//        telemetry.addLine()
//                .addData("status", new Func<String>() {
//                    @Override public String value() {
//                        return imu_gyro.getSystemStatus().toShortString();
//                    }
//                })
//                .addData("calib", new Func<String>) {
//                    @Override public String value () {
//                        return imu_gyro.getCalibrationStatus().toString();
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("heading", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.firstAngle);
//                    }
//                })
//                .addData("roll", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.secondAngle);
//                    }
//                })
//                .addData("pitch", new Func<String>() {
//                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, angles.thirdAngle);
//                    }
//                });
//
//        telemetry.addLine()
//                .addData("grvty", new Func<String>() {
//                    @Override public String value() {
//                        return gravity.toString();
//                    }
//                })
//                .addData("mag", new Func<String>() {
//                    @Override public String value() {
//                        return String.format(Locale.getDefault(), "%.3f",
//                                Math.sqrt(gravity.xAccel*gravity.xAccel
//                                        + gravity.yAccel*gravity.yAccel
//                                        + gravity.zAccel*gravity.zAccel));
//                    }
//                });
//    }

    String formatAngle( AngleUnit angleUnit, double angle) {
        return formatDegrees(angleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }



    public void setRunMode(SgpMotors eWhichMotor, DcMotor.RunMode eMode )
    {

        switch (eWhichMotor){
            case UPPER_LEFT:
                upper_left.setMode(eMode);
                break;
            case UPPER_RIGHT:
                upper_right.setMode(eMode);
                break;
            case LOWER_LEFT:
                lower_left.setMode(eMode);
                break;
            case LOWER_RIGHT:
                lower_right.setMode(eMode);
                break;
            case ARM_MOTOR:
                Arm_Motor.setMode(eMode);
                break;
            case INTAKE:
                Intake.setMode(eMode);
                break;
            case BRAVO6:
                Bravo_6.setMode(eMode);
                break;
            case BATMAN_BELT:
                Batman_Belt.setMode(eMode);
                break;
            case ALL:
                lower_right.setMode(eMode);
                lower_left.setMode(eMode);
                upper_right.setMode(eMode);
                upper_left.setMode(eMode);
                Arm_Motor.setMode(eMode);
                Batman_Belt.setMode(eMode);
                Bravo_6.setMode(eMode);
                break;
            case ALL_DRIVES:
                lower_right.setMode(eMode);
                lower_left.setMode(eMode);
                upper_right.setMode(eMode);
                upper_left.setMode(eMode);
                break;
        }
    }

    public void setPower(SgpMotors eWhichMotor, double dPower )
    {

        switch (eWhichMotor){
            case UPPER_LEFT:
                upper_left.setPower(dPower);
                break;
            case UPPER_RIGHT:
                upper_right.setPower(dPower);
                break;
            case LOWER_LEFT:
                lower_left.setPower(dPower);
                break;
            case LOWER_RIGHT:
                lower_right.setPower(dPower);
                break;
            case ARM_MOTOR:
                Arm_Motor.setPower(dPower);
                break;
            case INTAKE:
                Intake.setPower(dPower);
                break;
            case BATMAN_BELT:
                Batman_Belt.setPower(dPower);
                break;
            case BRAVO6:
                Bravo_6.setPower(dPower);
                break;
            case ALL:
                lower_right.setPower(dPower);
                lower_left.setPower(dPower);
                upper_right.setPower(dPower);
                upper_left.setPower(dPower);
                Arm_Motor.setPower(dPower);
                Batman_Belt.setPower(dPower);
                Bravo_6.setPower(dPower);
                break;
            case ALL_DRIVES:
                lower_right.setPower(dPower);
                lower_left.setPower(dPower);
                upper_right.setPower(dPower);
                upper_left.setPower(dPower);
                break;
        }
    }

    public int getCurrentPosition( SgpMotors eWhichMotor )
    {
        switch(eWhichMotor)
        {
            case UPPER_LEFT:
                return upper_left.getCurrentPosition();
            case LOWER_LEFT:
                return lower_left.getCurrentPosition();
            case UPPER_RIGHT:
                return upper_right.getCurrentPosition();
            case LOWER_RIGHT:
                return lower_right.getCurrentPosition();
            case ARM_MOTOR:
                return Arm_Motor.getCurrentPosition();
            case INTAKE:
                return Intake.getCurrentPosition();
            case BRAVO6:
                return Bravo_6.getCurrentPosition();
            case BATMAN_BELT:
                return Batman_Belt.getCurrentPosition();
            case ALL:
            case ALL_DRIVES:
            default:
                return 0;
        }
    }

    public void setTargetPosition( SgpMotors eWhichMotor, int iPos )
    {
        switch( eWhichMotor)
        {
            case UPPER_LEFT:
                upper_left.setTargetPosition(iPos);
                break;
            case LOWER_LEFT:
                lower_left.setTargetPosition(iPos);
                break;
            case UPPER_RIGHT:
                upper_right.setTargetPosition(iPos);
                break;
            case LOWER_RIGHT:
                lower_right.setTargetPosition(iPos);
                break;
            case INTAKE:
                Intake.setTargetPosition(iPos);
                break;
            case BATMAN_BELT:
                Batman_Belt.setTargetPosition(iPos);
                break;
            case BRAVO6:
                Bravo_6.setTargetPosition(iPos);
                break;
            case ARM_MOTOR:
                Arm_Motor.setTargetPosition(iPos);
                break;
            case ALL:
            case ALL_DRIVES:
            default :
                break;
        }
    }

    public boolean areMotorsBusy(SgpMotors eWhichMotor) {

        switch(eWhichMotor)
        {
            case UPPER_LEFT: // upper left
                return upper_left.isBusy();
            case LOWER_LEFT: // lower left
                return lower_left.isBusy();
            case UPPER_RIGHT: // upper right
                return upper_right.isBusy();
            case LOWER_RIGHT: // lower right
                return lower_right.isBusy();
            case ARM_MOTOR:
                return  Arm_Motor.isBusy();
            case INTAKE:
                return Intake.isBusy();
            case BRAVO6:
                return Bravo_6.isBusy();
            case BATMAN_BELT:
                return Batman_Belt.isBusy();
            case ALL_DRIVES: // All Drives
                return lower_left.isBusy() && lower_right.isBusy() && upper_left.isBusy() && upper_right.isBusy();
            default:
                return false;
        }
    }

 }

