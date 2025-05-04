package com.team5817.lib.drivers;

import com.team5817.lib.drivers.State.ServoState;

public class StateBasedServoMotorSubsystemWithCancoder<S extends Enum<S>&ServoState> extends ServoMotorSubsystemWithCancoder {
    private S mState;
    private final boolean allowAutoStateOutput;

    public StateBasedServoMotorSubsystemWithCancoder(ServoMotorSubsystemConstants constants, AbsoluteEncoderConstants encoderConstants, S initialState, boolean enableAutoStateOutput) {
        super(constants,encoderConstants);
        this.mState = initialState;
        this.allowAutoStateOutput = enableAutoStateOutput;
    }
    public StateBasedServoMotorSubsystemWithCancoder(ServoMotorSubsystemConstants constants,AbsoluteEncoderConstants encoderConstants, S initialState) {
        this(constants, encoderConstants, initialState, true);
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
