package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hooks {
    private CRServo hooks;
    private boolean wantsRetract = true;
    private Gamepad operator;
    private ElapsedTime timer;

    public Hooks(HardwareMap hardwareMap, Gamepad operator) {
        this.hooks = hardwareMap.get(CRServo.class, "hooks");
        this.operator = operator;
        this.timer = new ElapsedTime();
    }

    public void retract() {
        if (operator.b) { wantsRetract = !wantsRetract; }

        double startTime = timer.milliseconds();
        while (startTime - timer.milliseconds() < 0.5) {
            hooks.setPower(wantsRetract ? -1.0 : 1.0);
        }
    }
}
