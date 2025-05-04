package com.team5817.lib.drivers;

import com.team5817.lib.drivers.State.ServoState;

public class StateBasedServoMotorSubsystem<S extends Enum<S>&ServoState> extends ServoMotorSubsystem {
    private S mState;
    private final boolean allowAutoStateOutput;

    public StateBasedServoMotorSubsystem(ServoMotorSubsystemConstants constants, S initialState, boolean enableAutoStateOutput) {
        super(constants);
        this.mState = initialState;
        this.allowAutoStateOutput = enableAutoStateOutput;
    }
    public StateBasedServoMotorSubsystem(ServoMotorSubsystemConstants constants, S initialState) {
        this(constants, initialState, true);
    }

    public void setState(S state) {
        mState = state;
    }

    public S getState() {
        return mState;
    }

    @Override
    public void writePeriodicOutputs() {
        if(mControlState == ControlState.MOTION_MAGIC&&allowAutoStateOutput) 
            super.setSetpointMotionMagic(kMotionMagicSlot, mState.getDesiredPosition());
        super.writePeriodicOutputs();
    }
}
