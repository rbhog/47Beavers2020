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

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Capstone;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Hooks;
import org.firstinspires.ftc.teamcode.util.DriveMotion;
import org.firstinspires.ftc.teamcode.util.ElevatorMotion;
import org.firstinspires.ftc.teamcode.util.Motion;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.OtherMotion;

@TeleOp(name = "BigBoyDriversOnly", group = "TeleOpModes")
public class BigBoyDriving extends LinearOpMode {
    private ElapsedTime timer;

    private Drivetrain drive;
    private DriveMotion motion;
    private Elevator elevator;
    private ElevatorMotion elevatorMotion;
    private Hooks hooks;
    private Intake intake;
    private Capstone capstone;

    public void runOpMode() {
        timer = new ElapsedTime();
        drive = new Drivetrain(hardwareMap, timer);
        motion = new DriveMotion(drive);
        elevator = new Elevator(hardwareMap);
        elevatorMotion = new ElevatorMotion(elevator);
        intake = new Intake(hardwareMap);
        hooks = new Hooks(hardwareMap);
        capstone = new Capstone(hardwareMap);

        boolean wantsReverse = false;
        boolean wantsDown = false;

        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            hooks.actuate(-0.7);
            sleep(700);
            hooks.actuate(0.0);

            Motion movement = motion.rightwardsMotion(gamepad1.left_stick_x).add(motion.forwardMotion(-gamepad1.left_stick_y)).add(motion.rotationMotion(gamepad1.right_stick_x * (!wantsReverse ? 1.0 : -1.0)));

            if (gamepad1.dpad_left) {
                movement = movement.add(motion.forwardMotion(-0.3));
            }
            if (gamepad1.dpad_right) {
                movement = movement.add(motion.forwardMotion(0.3));
            }
            if (gamepad1.dpad_up) {
                movement = movement.add(motion.forwardMotion(-0.6));
            }
            if (gamepad1.dpad_down) {
                movement = movement.add(motion.forwardMotion(0.6));
            }

            if (gamepad1.x) {
                wantsDown = !wantsDown;
                if (wantsDown) {
                    hooks.actuate(0.7);
                    sleep(100);
                } else {
                    hooks.actuate(-0.7);
                    sleep(100);
                }
                hooks.actuate(0.0);
            }

            while (gamepad1.right_trigger > 0) {
                intake.actuate(0.5);
            }
            while (gamepad1.left_trigger > 0) {
                intake.actuate(-0.5);
            }

            if (!(gamepad1.right_trigger > 0) && !(gamepad1.left_trigger > 0)) {
                intake.actuate(0.0);
            }

            if (gamepad1.a) {
                wantsReverse = !wantsReverse;
                drive.reverse(wantsReverse);
            }

            capstone.actuate(gamepad2.left_stick_x);

            OtherMotion elevatorMovement = elevatorMotion.upwardsMotion(gamepad2.left_stick_y*.5).add(elevatorMotion.outwardsMotion(-gamepad2.left_stick_x*.5));

            motion.executeRate(movement);
            elevatorMotion.executeRate(elevatorMovement);


        }
    }
}
