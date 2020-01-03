/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.Motion;
import org.firstinspires.ftc.teamcode.util.Motor;
import org.firstinspires.ftc.teamcode.util.DriveMotion;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Drivetrain {
    private Motor[] motors;

    private int frontRight = 0;
    private int frontLeft = 2;
    private int backRight = 1;
    private int backLeft = 3;

    private BNO055IMU gyro;
    private double globalAngle;

    private Orientation lastAngles = new Orientation();

    private final double WHEEL_RADIUS = 4.0; //inches
    private final double PROPORTIONAL_CONSTANT = 0.2;
    private final double TICKS_PER_ROTATION = 288; //need to test

    private final double MAX_MOTOR_RATE = 1.0;

    private double globalX = 0.0, globalY = 0.0;

    private ElapsedTime timer;

    public Drivetrain(HardwareMap hardwareMap, ElapsedTime timer) {
        motors = new Motor[4];

        for (int i = 0; i < 4; ++i) {
            motors[i] = new Motor(
                    hardwareMap.get(DcMotor.class, "motor" + Integer.toString(i + 1)),
                    PROPORTIONAL_CONSTANT,
                    TICKS_PER_ROTATION
            );
        }

        motors[backRight].setDirection(DcMotor.Direction.REVERSE);
        motors[backLeft].setDirection(DcMotor.Direction.FORWARD);
        motors[frontRight].setDirection(DcMotor.Direction.REVERSE);
        motors[frontLeft].setDirection(DcMotor.Direction.FORWARD);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        gyro = hardwareMap.get(BNO055IMU.class, "imu");

        gyro.initialize(parameters);

        this.timer = timer;

    }

    public Motor getMotor(int index) {
        return motors[index];
    }

    public void update() {
        for (Motor motor : motors) {
            motor.proportionalUpdate();
        }
    }

    /**
     * Goes to the given coordinates
     *
     * @param x The ticks in the x-direction of the wanted location
     * @param y The ticks in the y-direction of the wanted location
     */
    public void driveToPosition(double x, double y) {
        DriveMotion motion = new DriveMotion(this);
        double targetAngle = Math.atan(y / x);

        while (globalX - x > 2) {
            motion.executeRate(motion.rightwardsMotion(-0.4), 0.2, Motion.dir.S);
        }
        while (globalX - x < -2) {
            motion.executeRate(motion.rightwardsMotion(0.4), 0.2, Motion.dir.S);
        }
        while (globalY - y > 2) {
            motion.executeRate(motion.forwardMotion(-0.4), 0.2, Motion.dir.F);
        }
        while (globalY - y < -2) {
            motion.executeRate(motion.forwardMotion(0.4), 0.2, Motion.dir.F);
        }
    }


    public double getTicksPerInch() {
        return TICKS_PER_ROTATION / (2 * Math.PI * WHEEL_RADIUS);
    }

    public double getMaxMotorRate() {
        return MAX_MOTOR_RATE;
    }

    public int getFrontRight() {
        return frontRight;
    }

    public int getFrontLeft() {
        return frontLeft;
    }

    public int getBackRight() {
        return backRight;
    }

    public int getBackLeft() {
        return backLeft;
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection(double targetAngle)
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == targetAngle)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public double getGlobalX() {
        return globalX;
    }

    public void setGlobalX(double globalX) {
        this.globalX = globalX;
    }

    public double getGlobalY() {
        return globalY;
    }

    public void setGlobalY(double globalY) {
        this.globalY = globalY;
    }

    /**
     * Stop all actions for a specified amount of time (in milliseconds)
     * @param milliseconds amount of time to wait
     */
    public void waitMilliseconds(double milliseconds){
        //Reset the timer
        timer.reset();
        //Wait until the time inputted has fully elapsed
        while(timer.milliseconds() < milliseconds);
    }

    public ElapsedTime getTimer() {
        return this.timer;
    }
}
