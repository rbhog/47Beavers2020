package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hooks {
    private CRServo[] hooks;

    private int hooksRight = 0;
    private int hooksLeft = 1;

    public Hooks(HardwareMap hardwareMap) {
        hooks = new CRServo[2];

        for (int i = 0; i < 2; i++) {
            hooks[i] = hardwareMap.get(CRServo.class, "hooks" + Integer.toString(i + 1));
        }

        hooks[hooksRight].setDirection(DcMotorSimple.Direction.REVERSE);
        hooks[hooksLeft].setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void actuate(long x) {
        for (int i = 0; i < 2; i++) {
            hooks[i].setPower(0.7);
        }

        try {
            Thread.sleep(x);
        } catch (InterruptedException e) {

        }

    }


}
