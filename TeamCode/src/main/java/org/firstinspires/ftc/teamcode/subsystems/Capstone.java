package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Capstone {
    private CRServo capstone;

    public Capstone(HardwareMap hardwareMap) {
        capstone = hardwareMap.get(CRServo.class, "capstone");

        capstone.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void actuate(double power) {
        capstone.setPower(power);
    }
}
