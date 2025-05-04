package com.team5817.lib.drivers;

import com.team5817.lib.drivers.State.RollerState;
import com.team5817.lib.requests.Request;

import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class StateBasedRollerSubsystem<S extends Enum<S> & RollerState> extends RollerSubsystem {
    @Getter @Setter @Accessors(prefix = "m") protected S mState;

    public StateBasedRollerSubsystem(S initialState, RollerSubsystemBasic.RollerSubsystemConstants... constantsArray ) {
        super(constantsArray);
        this.mState = initialState;
    }

    @Override
    public void writePeriodicOutputs() {
        for (int i = 0; i < rollers.size(); i++) {
            RollerSubsystemBasic roller = rollers.get(i);
            switch (mState.getControlStates()[i]) {
                case OPEN_LOOP:
                    roller.setOpenLoop(mState.getRollerDemands()[i]);
                    break;
                case VELOCITY:
                    roller.setVelocity(mState.getRollerDemands()[i]);
                    break;
                case VOLTAGE:
                    roller.applyVoltage(mState.getRollerDemands()[i]);
                    break;
                
            }
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
