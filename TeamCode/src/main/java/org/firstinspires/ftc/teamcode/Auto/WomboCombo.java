package org.firstinspires.ftc.teamcode.Auto;

import android.graphics.Bitmap;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Hooks;
import org.firstinspires.ftc.teamcode.util.DriveMotion;
import org.firstinspires.ftc.teamcode.util.ElevatorMotion;
import org.firstinspires.ftc.teamcode.util.SamplePipeline;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;


@Autonomous (name = "GOD IS DEAD AND WE HAVE KILLED HIM - HARAMBE", group = "Autonomous")
public class WomboCombo extends LinearOpMode {
    OpenCvCamera phoneCam;
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
        Rect[] rects = new Rect[3];
        Mat[] mats = new Mat[3];
        double[] diffs = new double[3];
        for(int i=0;i<3;i++){
            rects[i] = new Rect(startx + width * i, starty, width, height);
        }
        for(int i=0;i<3;i++){
            mats[i] = new Mat(wholeImage, rects[i]);
        }
        for(int i=0;i<3;i++){
            Mat m = mats[i];
            Mat bCnl = new Mat();
            Core.extractChannel(m, bCnl, 0);
            Mat gCnl = new Mat();
            Core.extractChannel(m, gCnl, 1);
            Mat rCnl = new Mat();
            Core.extractChannel(m, rCnl, 2);
            // get mean value
            double bMean = Core.mean(bCnl).val[0];
            double gMean = Core.mean(gCnl).val[0];
            double rMean = Core.mean(rCnl).val[0];
            diffs[i] = Math.abs(25 - bMean) + Math.abs(132-gMean) + Math.abs(213-rMean); 
        }
        //0 -> left, 1 -> middle, 2 -> right
        //coords might be wrong
        return diffs[0] > diffs[1] && diffs[0] > diffs[2] ? 0 : diffs[1]>diffs[2] ? 1 : 2;
    }

    private void saveImage(Mat subimg, String filename) {
        Bitmap bmp = null;
        try {
            bmp = Bitmap.createBitmap(subimg.cols(), subimg.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(subimg, bmp);
        } catch (CvException e) {
            Log.d("Autonomous vision", e.getMessage());
        }

        subimg.release();


        FileOutputStream out = null;



        File sd = new File(Environment.getExternalStorageDirectory() + "/frames");
        boolean success = true;
        if (!sd.exists()) {
            success = sd.mkdir();
        }
        if (success) {
            File dest = new File(sd, filename);

            try {
                out = new FileOutputStream(dest);
                bmp.compress(Bitmap.CompressFormat.PNG, 100, out); // bmp is your Bitmap instance
                // PNG is a lossless format, the compression factor (100) is ignored

            } catch (Exception e) {
                e.printStackTrace();
                Log.d("Autonomous vision", e.getMessage());
            } finally {
                try {
                    if (out != null) {
                        out.close();
                        Log.d("Autonomous vision", "OK!!");
                    }
                } catch (IOException e) {
                    Log.d("Autonomous vision", e.getMessage() + "Error");
                    e.printStackTrace();
                }
            }
        }
    }
    private Mat takePic(){
        ourPipeline.request();
        thread.sleep(100); //wait a decent amt of time...can be changed
        return ourPipeline.getImage();
    }
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
