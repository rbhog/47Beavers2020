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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic driving", group="Iterative Opmode")
//@Disabled
public class BasicDriving extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    //Declare motors and servoes
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor armRight = null;
    private DcMotor armLeft = null;

//    // declare motor speed variables
//    double RF; double LF; double RR; double LR;
//    // declare joystick position variables
//    double X1; double Y1; double X2; double Y2;


    //Driving constants
    final private double TURN_EXPONENT = 1.5;
    final private double LIFT_POWER = 0.6;

    // operational constants
    double joyScale = 1.0; //0.5;
    double motorMax = 1.0; //0.6; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode


//    //Driving mode (tank/POV)
//    final private int DRIVE_MODE_TANK = 1;
//    final private int DRIVE_MODE_STRAFE = 2;
//    final private int DRIVE_MODE_PIVOT = 3;

    //private int driveMode = DRIVE_MODE_TANK;

    private final boolean COMPETITION_MODE = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

//        armRight = hardwareMap.get(DcMotor.class, "arm_right");
//        armLeft = hardwareMap.get(DcMotor.class, "arm_left");
//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontRight.setDirection(DcMotor.Direction.FORWARD);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.FORWARD);


        //Encoder stuff
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        //Set driving powers
        double frontLeft = 0;
        double frontRight = 0;
        double backLeft = 0;
        double backRight = 0;

        //amount to change each motor by
        double drive = 0;
        double turn = 0;
        double strafe = 0;
        boolean pivot;

        //drive forewards and strafeing
//        drive = - gamepad1.left_stick_y;
//        telemetry.addData("$drive", drive);
//        strafe = - gamepad1.left_stick_x;
//        turn = - gamepad1.right_stick_x;
//        pivot = gamepad1.right_stick_button;
//
//
//
//        if(Math.abs(drive)>.05){
//            frontLeft = drive;
//            frontRight = drive;
//            backLeft = drive;
//            backRight = drive;
//        }
//        else if(Math.abs(strafe)>.05){
//            frontLeft = strafe;
//            frontRight = -strafe;
//            backLeft = -strafe;
//            backRight = strafe;
//        }
//        else if(Math.abs(turn)>.05){
//            if(pivot){
//                if(turn<0){
//                    frontRight = - turn;
//                    backRight = - turn;
//                }
//                else{
//                    frontLeft = turn;
//                    backLeft = turn;
//                }
//            }
//            else{
//                frontLeft = turn;
//                frontRight = -turn;
//                backLeft = turn;
//                backRight = -turn;
//            }
//        }

        // declare motor speed variables
        double RF; double LF; double RR; double LR;
        // declare joystick position variables
        double X1; double Y1; double X2; double Y2;

        LF = 0; RF = 0; LR = 0; RR = 0;

        // Get joystick values
        Y1 = -gamepad1.right_stick_y * joyScale; // invert so up is positive
        X1 = gamepad1.right_stick_x * joyScale;
        Y2 = -gamepad1.left_stick_y * joyScale; // Y2 is not used at present
        X2 = gamepad1.left_stick_x * joyScale;

        // Forward/back movement
        LF += Y1; RF += Y1; LR += Y1; RR += Y1;

        // Side to side movement
        LF += X1; RF -= X1; LR -= X1; RR += X1;

        // Rotation movement
        LF += X2; RF -= X2; LR += X2; RR -= X2;

        // Clip motor power values to +-motorMax
        LF = Math.max(-motorMax, Math.min(LF, motorMax));
        RF = Math.max(-motorMax, Math.min(RF, motorMax));
        LR = Math.max(-motorMax, Math.min(LR, motorMax));
        RR = Math.max(-motorMax, Math.min(RR, motorMax));

        this.frontRight.setPower(RF);
        this.frontLeft.setPower(LF);
        this.backRight.setPower(RR);
        this.backLeft.setPower(LR);


//        if (gamepad1.right_bumper) {
//            setArmPowers(LIFT_POWER);
//        } else if (gamepad1.left_bumper) {
//            setArmPowers(-LIFT_POWER);
//        }


        setDrivePowers(backLeft, backRight, frontLeft, frontRight);

        telemetry.update();
    }

    @Override
    public void stop() {
        if(COMPETITION_MODE){
            telemetry.clearAll();
            telemetry.addData("So the match's over", "How'd it go?");
        }
    }

    public void setArmPowers(double power) {
        armRight.setPower(power);
        armLeft.setPower(-power);
    }

    public void setDrivePowers(double backLeftPower, double backRightPower, double frontLeftPower, double frontRightPower) {
        backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
        backRightPower = Range.clip(backRightPower, -1.0, 1.0);
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);

        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
    }

//    public void setDrivePowers(double left, double right){
//        left = Range.clip(left, -1.0, 1.0);
//        right = Range.clip(right, -1.0, 1.0);
//        if(frontDirection == DIRECTION_HUB){
//            frontLeft.setPower(left);
//            frontRight.setPower(right);
//        } else if(frontDirection == DIRECTION_ARM){
//            frontLeft.setPower(-right);
//            frontRight.setPower(-left);
//        } else{
//            frontLeft.setPower(left);
//            frontRight.setPower(right);
//        }
//    }

}