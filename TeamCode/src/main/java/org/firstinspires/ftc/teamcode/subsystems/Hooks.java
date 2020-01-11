package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hooks {
    private CRServo[] hooks;

    private int hooksRight = 0;
    private int hooksLeft = 1;

    public Hooks(HardwareMap hardwareMap) {
        hooks = new CRServo[2];

        for (int i = 0; i < 2; i++) {
            hooks[i] = hardwareMap.get(CRServo.class, "hooks" + Integer.toString(i + 1));
        }

        hooks[hooksRight].setDirection(DcMotorSimple.Direction.FORWARD);
        hooks[hooksLeft].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void actuate(double power) {
        for (int i = 0; i < 2; i++) {
            hooks[i].setPower(power);
        }
//        try {
//            Thread.sleep(1000);
//        } catch (InterruptedException e) {}
//        for (int i = 0; i < 2; i++) {
//            hooks[i].setPower(0.0);
//        }




    }


}
