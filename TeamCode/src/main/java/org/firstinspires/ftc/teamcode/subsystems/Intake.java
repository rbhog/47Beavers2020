package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final CRServo motor;
    private final Gamepad operator;
    private boolean wantsRetract = false;

    public Intake(Telemetry telemetry1, HardwareMap hardwareMap, Gamepad gamepad) {
        this.telemetry = telemetry1;
        this.hardwareMap = hardwareMap;
        this.operator = gamepad;

        this.motor = hardwareMap.get(CRServo.class, "intake");
    }

    public void actuate() {
        if (operator.a) { wantsRetract = !wantsRetract; }

        //need a slower retract so that the frame does not get damaged
        motor.setPower(wantsRetract ? 1.0 : -0.75);
    }



}
