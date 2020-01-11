package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.DriveMotion;
import org.firstinspires.ftc.teamcode.util.Motion;

@Autonomous (name = "Foundation", group = "Autonomous")
public class Foundation extends LinearOpMode {
    private Drivetrain drivetrain;
    private ElapsedTime timer;
    private DriveMotion driveMotion;

    public void runOpMode() {
        timer = new ElapsedTime();
        drivetrain = new Drivetrain(hardwareMap, timer);
        driveMotion = new DriveMotion(drivetrain);

        waitForStart();

        while (opModeIsActive()) {
            //yeet across
            Motion movement = driveMotion.rotationMotion(-0.7);
            driveMotion.executeRate(movement);
            sleep(1400);
            movement = driveMotion.forwardMotion(1);
            driveMotion.executeRate(movement);
            sleep(2500);
            /*
            //go to foundation
            movement = driveMotion.rotationMotion(0.7);
            driveMotion.executeRate(movement);
            sleep(500);
            movement = driveMotion.forwardMotion(0.7);
            driveMotion.executeRate(movement);
            sleep(1500);
            //hooks down
            movement = driveMotion.forwardMotion(-0.7);
            driveMotion.executeRate(movement);
            sleep(750);
            movement = driveMotion.rotationMotion(-0.7);
            driveMotion.executeRate(movement);
            sleep(1000);
            movement = driveMotion.rightwardsMotion(-0.7);
            driveMotion.executeRate(movement);
            sleep(500);
            //hooks up
            movement = driveMotion.forwardMotion(-0.7);
            driveMotion.executeRate(movement);
            sleep(1500);
            //kill power
            */
            driveMotion.executeRate(driveMotion.forwardMotion(0.0));
            stop();
        }
    }

}
