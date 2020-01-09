package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class SamplePipeline extends OpenCvPipeline {
    private Mat currentImage;
    private boolean req=false;
    public void request() {
        req = true;
    }
    public Mat getImage(){
        return currentImage;
    }
    @Override
    public Mat processFrame(Mat input) {
        if(req){
            currentImage = input;
        }
        return null;
    }
}
