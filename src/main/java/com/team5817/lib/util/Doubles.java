package com.team5817.lib.util;

public final class Doubles {
    public static double[] of(Double... values) {
        double[] result = new double[values.length];
        for (int i = 0; i < values.length; i++) {
            result[i] = values[i];
        }
        return result;
    }
}

