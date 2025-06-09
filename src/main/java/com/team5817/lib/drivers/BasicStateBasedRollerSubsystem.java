package com.team5817.lib.drivers;

import com.team5817.lib.drivers.State.BasicRollerState;
import com.team5817.lib.requests.Request;

import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class BasicStateBasedRollerSubsystem<S extends Enum<S> & BasicRollerState> extends RollerSubsystemBasic {
    @Getter @Setter @Accessors(prefix = "m") protected S mState;

    public BasicStateBasedRollerSubsystem(S initialState, RollerSubsystemConstants constants) {
        super(constants); 
        this.mState = initialState;
    }

    @Override
    public void writePeriodicOutputs() {

        switch (mState.getControlState()) {
            case OPEN_LOOP:
                setOpenLoop(mState.getRollerDemand());
                break;
            case VELOCITY:
                setVelocity(mState.getRollerDemand());
                break;
            case VOLTAGE:
                setVoltage(mState.getRollerDemand());
                break;
            
        }

        super.writePeriodicOutputs();
    }
    
    public Request stateRequest(S _wantedState) {
        return new Request() {
            @Override
            public boolean isFinished() {
                return true;
            }

            @Override
            public void act() {
                setState(_wantedState);
            }
        };
    }
}
