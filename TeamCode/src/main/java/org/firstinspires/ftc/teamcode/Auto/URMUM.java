package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Hooks;
import org.firstinspires.ftc.teamcode.util.DriveMotion;
import org.firstinspires.ftc.teamcode.util.Motion;

@Autonomous(name = "ADHVAYITH USE THIS PLZ", group = "Autonomous")
public class URMUM extends LinearOpMode {
    private Drivetrain drivetrain;
    private ElapsedTime timer;

    private DriveMotion driveMotion;

    public void runOpMode() {
        timer = new ElapsedTime();
        drivetrain = new Drivetrain(hardwareMap, timer);
        driveMotion = new DriveMotion(drivetrain);

        waitForStart();

        while (opModeIsActive()) {
            Motion movement = driveMotion.rightwardsMotion(1.0);
            driveMotion.executeRate(movement);
            sleep(2900);
            driveMotion.executeRate(driveMotion.forwardMotion(0.0));
            stop();
        }
    }
}
