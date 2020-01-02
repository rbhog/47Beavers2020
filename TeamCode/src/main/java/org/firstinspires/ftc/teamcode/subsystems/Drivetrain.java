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

import org.firstinspires.ftc.teamcode.util.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {
    private Motor[] motors;

    private int frontRight = 0;
    private int frontLeft = 2;
    private int backRight = 1;
    private int backLeft = 3;

    private final double WHEEL_RADIUS = 4.0; //inches
    private final double PROPORTIONAL_CONSTANT = 0.2;
    private final double TICKS_PER_ROTATION = 288; //need to test

    private final double MAX_MOTOR_RATE = 1.0;

    public Drivetrain(HardwareMap hardwareMap) {
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

    }

    public Motor getMotor(int index) {
        return motors[index];
    }

    public void update() {
        for (Motor motor : motors) {
            motor.proportionalUpdate();
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
}
