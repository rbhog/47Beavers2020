package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hooks {
    private Servo[] hooks;

    private int hooksRight = 0;
    private int hooksLeft = 1;

    public Hooks(HardwareMap hardwareMap) {
        hooks = new Servo[2];

        for (int i = 0; i < 2; i++) {
            hooks[i] = hardwareMap.get(Servo.class, "hooks" + Integer.toString(i + 1));
        }

        hooks[hooksRight].setDirection(Servo.Direction.FORWARD);
        hooks[hooksLeft].setDirection(Servo.Direction.REVERSE);
    }

    public void actuate(boolean wantsDown) {
        for (int i = 0; i < 2; i++) {
            hooks[i].setPosition(180.0);
        }


    }


}
