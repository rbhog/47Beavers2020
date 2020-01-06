package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;

public class ElevatorMotion {
    private Elevator elevator;

    public ElevatorMotion(Elevator elevator) {
        this.elevator = elevator;
    }

    public void executeRate(OtherMotion motion) {
        for (int i = 0; i < 2; i++) {
            elevator.getMotor(i).setRate(motion.getValue(i) * elevator.getMaxMotorRate());
        }

        for (int i = 0; i < 2; i++) {
            elevator.getServo(i).setPower(motion.getValue(i + 2));
        }
    }

    public OtherMotion upwardsMotion(double x) {
        double[] values = new double[4];
        values[0] = x;
        values[1] = x;
        values[2] = 0;
        values[3] = 0;
        return new OtherMotion(values);
    }

    public OtherMotion outwardsMotion(double x) {
        double[] values = new double[4];
        values[0] = 0;
        values[1] = 0;
        values[2] = x;
        values[3] = x;
        return new OtherMotion(values);
    }
}
