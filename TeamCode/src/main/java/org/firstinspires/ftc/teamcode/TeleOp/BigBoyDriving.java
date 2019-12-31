package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.DriveMotion;
import org.firstinspires.ftc.teamcode.util.Motion;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "BigBoyDriversOnly", group = "TeleOpModes")
public class BigBoyDriving extends LinearOpMode {
    private Drivetrain drive;
    private DriveMotion motion;
    private Intake intake;

    public void runOpMode() {
        drive = new Drivetrain(hardwareMap);
        motion = new DriveMotion(drive);

        waitForStart();

        boolean flag = false;

        while (opModeIsActive()) {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            Motion movement = motion.rightwardsMotion(gamepad1.left_stick_x).add(motion.forwardMotion(-gamepad1.left_stick_y)).add(motion.rotationMotion(gamepad1.right_stick_x));

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

            motion.executeRate(movement);


        }
    }
}
