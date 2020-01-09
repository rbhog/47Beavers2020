package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Hooks;
import org.firstinspires.ftc.teamcode.util.DriveMotion;
import org.firstinspires.ftc.teamcode.util.ElevatorMotion;
import org.firstinspires.ftc.teamcode.util.SamplePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


@Autonomous (name = "GOD IS DEAD AND WE HAVE KILLED HIM - HARAMBE", group = "Autonomous")
public class WomboCombo extends LinearOpMode {
    OpenCvCamera phoneCam;
    private ElapsedTime timer;
    private Drivetrain drivetrain;
    private Elevator elevator;
    private Hooks hooks;

    private DriveMotion driveMotion;
    private ElevatorMotion elevatorMotion;

//    private int selectPath(Mat wholeImage){
//        //need to be set
//        int startx=0;
//        int starty=0;
//        int width=0;
//        int height=0;
//
//        Rect block1 = new Rect(startx, starty, width, height);
//        Rect block2 = new Rect(startx + width, starty, width, height);
//        Rect block3 = new Rect(startx + width*2, starty, width, height);
//
//        Mat img1 = new Mat(wholeImage, block1);
//        Mat img2 = new Mat(wholeImage, block2);
//        Mat img3 = new Mat(wholeImage, block3);
//
//        return 0;
//    }
    public void runOpMode() {
        timer = new ElapsedTime();
        drivetrain = new Drivetrain(hardwareMap, timer);
        elevator = new Elevator(hardwareMap);
        hooks = new Hooks(hardwareMap);

        driveMotion = new DriveMotion(drivetrain);
        elevatorMotion = new ElevatorMotion(elevator);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        SamplePipeline ourPipeline = new SamplePipeline();

        phoneCam.setPipeline(ourPipeline);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.update();


        }
    }


}
