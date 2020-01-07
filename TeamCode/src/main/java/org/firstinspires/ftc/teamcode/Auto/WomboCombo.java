package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Hooks;
import org.firstinspires.ftc.teamcode.util.DriveMotion;
import org.firstinspires.ftc.teamcode.util.ElevatorMotion;
import org.opencv.core.Mat;
import org.opencv.core.Rect;


@Autonomous (name = "GOD IS DEAD AND WE HAVE KILLED HIM - HARAMBE", group = "Autonomous")
public class WomboCombo extends LinearOpMode {

    private ElapsedTime timer;
    private Drivetrain drivetrain;
    private Elevator elevator;
    private Hooks hooks;

    private DriveMotion driveMotion;
    private ElevatorMotion elevatorMotion;

    private int selectPath(Mat wholeImage){
        //need to be set
        int startx=0;
        int starty=0;
        int width=0;
        int height=0;

        Rect block1 = new Rect(startx, starty, width, height);
        Rect block2 = new Rect(startx + width, starty, width, height);
        Rect block3 = new Rect(startx + width*2, starty, width, height);

        Mat img1 = new Mat(wholeImage, block1);
        Mat img2 = new Mat(wholeImage, block2);
        Mat img3 = new Mat(wholeImage, block3);

        return 0;
    }
    public void runOpMode() {
        timer = new ElapsedTime();
        drivetrain = new Drivetrain(hardwareMap, timer);
        elevator = new Elevator(hardwareMap);
        hooks = new Hooks(hardwareMap);

        driveMotion = new DriveMotion(drivetrain);
        elevatorMotion = new ElevatorMotion(elevator);

        waitForStart();

        while (opModeIsActive()) {

        }
    }


}
