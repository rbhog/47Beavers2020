package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Motor {
    private final DcMotor motor;
    private final double proportionalConstant;

    private final ElapsedTime timer;

    private double rate = 0; // ticks / milli

    private double[] previousTimes;
    private double[] previousTicks;

    private int STEPS = 2;

    public final double ticksPerRotation;

    public Motor(DcMotor motor, double proportionalConstant, double ticksPerRotation) {
        this.motor = motor;
        this.proportionalConstant = proportionalConstant;
        this.ticksPerRotation = ticksPerRotation;

        timer = new ElapsedTime();

        motor.setPower(0.0);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        previousTicks = new double[STEPS];
        previousTimes = new double[STEPS];
        for (int i = 0; i < STEPS; i++) {
            previousTimes[i] = timer.milliseconds();
            previousTicks[i] = this.motor.getCurrentPosition();
        }
    }

    public int getEncoderPosition() {
        return motor.getCurrentPosition();
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public void setRawPower(double power) {
        motor.setPower(power);
    }

    public void setRate(double ticksPerMilli) {
//        motor.setPower(motor.getPower() * ticksPerMilli / rate);
        rate = ticksPerMilli;
        proportionalUpdate();
    }

    public void proportionalUpdate() {
        double timeDelta = timer.milliseconds() - previousTimes[0];
        double tickDelta = getEncoderPosition() - previousTicks[0];
        double currentRate = tickDelta / timeDelta;
        double rateDelta = rate - currentRate;
        double power = motor.getPower() + rateDelta * proportionalConstant;
        double newPower = Math.min(-1.0, Math.max(power, 1.0));
        motor.setPower(newPower);

        motor.setPower(rate);

        for (int i = 0; i < STEPS - 1; i++) {
            previousTimes[i] = previousTimes[i + 1];
            previousTicks[i] = previousTicks[i + 1];
        }

        previousTimes[STEPS - 1] = timer.milliseconds();
        previousTicks[STEPS - 1] = getEncoderPosition();

    }
}
