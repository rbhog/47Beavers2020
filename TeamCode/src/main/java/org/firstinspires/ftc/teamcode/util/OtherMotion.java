package org.firstinspires.ftc.teamcode.util;

public class OtherMotion {
    private double[] values = new double[4];

    final int LENGTH = 4;

    public OtherMotion() {}

    public OtherMotion(double[] values) {
        this.values = values;
    }

    public OtherMotion add(OtherMotion other) {
        double[] out = new double[LENGTH];
        for (int i = 0; i < LENGTH; i++) {
            out[i] = values[i] + other.getValue(i);
        }
        return new OtherMotion(out);
    }

    public OtherMotion scale(double x) {
        double[] out = new double[LENGTH];
        for (int i = 0; i < LENGTH; i++) {
            out[i] = values[i] * x;
        }
        return new OtherMotion(out);
    }

    double getValue(int i) {
        return values[i];
    }



}
