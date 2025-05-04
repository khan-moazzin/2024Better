package com.team5817.lib.drivers;

public class State {
    public interface RollerState {
        double[] getRollerDemands();
        RollerSubsystemBasic.ControlState[] getControlStates();
    }
    public interface BasicRollerState {
        double getRollerDemand();
        RollerSubsystemBasic.ControlState getControlState();
    }

    public interface ServoState {
        double getDemand();
        boolean isDisabled();
        double getAllowableError();
        ServoMotorSubsystem.ControlState getControlState();
    }
}