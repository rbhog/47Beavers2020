package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

public class DriveMotion {

    private Drivetrain drive;

    public DriveMotion(Drivetrain robot) {
        this.drive = robot;
    }

    public void moveRate(double rate, double degrees) {
        double radians = Math.toRadians(degrees);
        Motion motion = forwardMotion(rate * Math.cos(radians))
                .add(rightwardsMotion(rate * Math.sin(radians)));

        executeRate(motion);
    }


    public void moveDistance(double distance, double rate, double degrees) {
        double radians = Math.toRadians(degrees);
        Motion motion = forwardMotion(distance * drive.getTicksPerInch() * Math.cos(radians))
                .add(rightwardsMotion(distance * drive.getTicksPerInch() * Math.sin(radians)));

        executeDistance(motion, rate);
    }

    public void rotate(double degrees) {

    }

    public void executeRate(Motion motion) {
        for (int i = 0; i != 4; ++i) {
//            drive.getDrive(i).setRate(motion.getValue(i));
            drive.getMotor(i).setRate(motion.getValue(i) * drive.getMaxMotorRate());
        }
    }

    public void executeDistance(Motion motion, double rate) {
        // each element of the motion is a number of ticks!

        Motor motor = null;
        for (int i = 0; i != 4; ++i) {
            if (motion.getValue(i) != 0) {
                motor = drive.getMotor(i);
            }
        }
        if (motor == null) {
            return;
        }
    }

    public Motion forwardMotion(double x) {
        double[] values = new double[4];
        values[drive.getFrontLeft()] = x;
        values[drive.getFrontRight()] = x;
        values[drive.getBackLeft()] = x;
        values[drive.getBackRight()] = x;
        return new Motion(values);
    }

    public Motion rightwardsMotion(double x) {
        double[] values = new double[4];
        values[drive.getFrontLeft()] = x;
        values[drive.getFrontRight()] = -x;
        values[drive.getBackLeft()] = -x;
        values[drive.getBackRight()] = x;
        return new Motion(values);
    }

    public Motion rotationMotion(double x) {
        double[] values = new double[4];
        values[drive.getFrontLeft()] = x;
        values[drive.getFrontRight()] = -x;
        values[drive.getBackLeft()] = x;
        values[drive.getBackRight()] = -x;
        return new Motion(values);
    }
}
