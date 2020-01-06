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

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Motor;

public class Elevator {
    private Motor[] motors;
    private CRServo[] outs;

    private int elevatorRight = 0;
    private int elevatorLeft = 1;
    private int elevatorOutRight = 2;
    private int elevatorOutLeft = 3;

    private final double PROPORTIONAL_CONSTANT = 0.2;
    private final double TICKS_PER_ROTATION = 1440;

    private final double MAX_MOTOR_RATE = 1.0;

    public Elevator(HardwareMap hardwareMap) {
        motors = new Motor[2];
        outs = new CRServo[2];

        for (int i = 0; i < 2; i++) {
            motors[i] = new Motor(
                    hardwareMap.get(DcMotor.class, "elevator" + Integer.toString(i + 1)),
                    PROPORTIONAL_CONSTANT,
                    TICKS_PER_ROTATION
            );
        }

        motors[elevatorRight].setDirection(DcMotorSimple.Direction.FORWARD);
        motors[elevatorLeft].setDirection(DcMotorSimple.Direction.REVERSE);

        for (int i = 0; i < 2; i++) {
            outs[i] = hardwareMap.get(CRServo.class, "elevator" + Integer.toString(i + 3));
        }

        outs[elevatorOutLeft - 2].setDirection(DcMotorSimple.Direction.FORWARD);
        outs[elevatorOutRight - 2].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getMaxMotorRate() {
        return MAX_MOTOR_RATE;
    }

    public int getElevatorRight() {
        return elevatorRight;
    }

    public int getElevatorLeft() {
        return elevatorLeft;
    }

    public int getElevatorOutRight() {
        return elevatorOutRight;
    }

    public int getElevatorOutLeft() {
        return elevatorOutLeft;
    }

//    public int getElevatorOut() {
//        return elevatorOut;
//    }

    public Motor getMotor(int index) { return motors[index]; }

    public CRServo getServo(int index) { return outs[index]; }
}
