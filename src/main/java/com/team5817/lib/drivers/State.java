package com.team5817.lib.drivers;

public class State {
    public interface RollerState {
        double[] getRollerVoltages();
    }
    public interface ServoState {
        double getDesiredPosition();
        boolean isDisabled();
        double getAllowableError();
    }
}