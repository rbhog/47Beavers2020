package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {
    private Motor[] motors;

    private int frontRight = 0;
    private int frontLeft = 2;
    private int backRight = 1;
    private int backLeft = 3;

    private final double WHEEL_RADIUS = 4.0; //inches
    private final double PROPORTIONAL_CONSTANT = 0.2;
    private final double TICKS_PER_ROTATION = 288; //need to test

    private final double MAX_MOTOR_RATE = 1.0;

    public Drivetrain(HardwareMap hardwareMap) {
        motors = new Motor[4];

        for (int i = 0; i < 4; ++i) {
            motors[i] = new Motor(
                    hardwareMap.get(DcMotor.class, "motor" + Integer.toString(i + 1)),
                    PROPORTIONAL_CONSTANT,
                    TICKS_PER_ROTATION
            );
        }

        motors[backRight].setDirection(DcMotor.Direction.REVERSE);
        motors[backLeft].setDirection(DcMotor.Direction.FORWARD);
        motors[frontRight].setDirection(DcMotor.Direction.REVERSE);
        motors[frontLeft].setDirection(DcMotor.Direction.FORWARD);

    }

    public Motor getMotor(int index) {
        return motors[index];
    }

    public void update() {
        for (Motor motor : motors) {
            motor.proportionalUpdate();
        }
    }

    public double getTicksPerInch() {
        return TICKS_PER_ROTATION / (2 * Math.PI * WHEEL_RADIUS);
    }

    public double getMaxMotorRate() {
        return MAX_MOTOR_RATE;
    }

    public int getFrontRight() {
        return frontRight;
    }

    public int getFrontLeft() {
        return frontLeft;
    }

    public int getBackRight() {
        return backRight;
    }

    public int getBackLeft() {
        return backLeft;
    }
}
