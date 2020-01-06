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

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.concurrent.TimeUnit;

public class DriveMotion {

    private Drivetrain drive;

    private ElapsedTime timer;
    private ElapsedTime moveTimer = new ElapsedTime();

    private double[] previousTimes;
    private double[] previousTicks;

    private final int STEPS = 2;


    public DriveMotion(Drivetrain robot) {
        this.drive = robot;
        this.timer = drive.getTimer();

        previousTicks = new double[STEPS];
        previousTimes = new double[STEPS];
        for (int i = 0; i < STEPS; i++) {
            previousTimes[i] = timer.milliseconds();
            previousTicks[i] = this.drive.getMotor(0).getEncoderPosition();
        }
    }

    public void moveRate(double rate, double degrees) {
        double radians = Math.toRadians(degrees);
        Motion motion = forwardMotion(rate * Math.cos(radians))
                .add(rightwardsMotion(rate * Math.sin(radians)));

        executeRate(motion);
    }


    public void moveDistance(double distance, double rate, double degrees) {
        double radians = Math.toRadians(degrees);
        Motion motion = forwardMotion(distance * drive.getTicksPerInch() * Math.cos(radians))
                .add(rightwardsMotion(distance * drive.getTicksPerInch() * Math.sin(radians)));

        executeDistance(motion, rate);
    }

    public void rotate(double degrees) {

    }

    public void executeRate(Motion motion) {
        for (int i = 0; i != 4; ++i) {
//            drive.getDrive(i).setRate(motion.getValue(i));
            if (motion.getValue(i) > 0.05 || motion.getValue(i) < -0.05) {
                drive.getMotor(i).setRate(motion.getValue(i) * drive.getMaxMotorRate());
            }
        }
    }

    public void executeRate(Motion motion, double time, Motion.dir dir) {
        for (int i = 0; i != 4; ++i) {
            drive.getMotor(i).setRate(motion.getValue(i) * drive.getMaxMotorRate());
        }

        while (moveTimer.milliseconds() < time) {
            for (int i = 0; i < STEPS - 1; i++) {
                previousTimes[i] = previousTimes[i + 1];
                previousTicks[i] = previousTicks[i + 1];
            }

            previousTimes[STEPS - 1] = timer.milliseconds();
            previousTicks[STEPS - 1] = this.drive.getMotor(0).getEncoderPosition();

            switch (dir) {
                case F:
                    drive.setGlobalY(drive.getGlobalX() + previousTicks[STEPS] - previousTicks[0]);
                    break;
                case S:
                    drive.setGlobalX(drive.getGlobalX() + previousTicks[STEPS] - previousTicks[0]);
                    break;
                default:
                    break;
            }
        }
    }


    public void executeDistance(Motion motion, double rate) {
        // each element of the motion is a number of ticks!

        Motor motor = null;
        for (int i = 0; i != 4; ++i) {
            if (motion.getValue(i) != 0) {
                motor = drive.getMotor(i);
            }
        }
        if (motor == null) {
            return;
        }
    }

    public Motion forwardMotion(double x) {
        double[] values = new double[4];
        values[drive.getFrontLeft()] = x;
        values[drive.getFrontRight()] = x;
        values[drive.getBackLeft()] = x;
        values[drive.getBackRight()] = x;

        drive.getMotor(0).getEncoderPosition();

        return new Motion(values, Motion.dir.F);

    }

    public Motion rightwardsMotion(double x) {
        double[] values = new double[4];
        values[drive.getFrontLeft()] = x;
        values[drive.getFrontRight()] = -x;
        values[drive.getBackLeft()] = -x;
        values[drive.getBackRight()] = x;
        return new Motion(values, Motion.dir.S);
    }

    public Motion rotationMotion(double x) {
        double[] values = new double[4];
        values[drive.getFrontLeft()] = x;
        values[drive.getFrontRight()] = -x;
        values[drive.getBackLeft()] = x;
        values[drive.getBackRight()] = -x;
        return new Motion(values, Motion.dir.R);
    }
}
