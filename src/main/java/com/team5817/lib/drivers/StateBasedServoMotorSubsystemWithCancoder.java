package com.team5817.lib.drivers;

import org.littletonrobotics.junction.Logger;

import com.team5817.lib.Util;
import com.team5817.lib.drivers.State.ServoState;
import com.team5817.lib.requests.Request;

public class StateBasedServoMotorSubsystemWithCancoder<S extends Enum<S>&ServoState> extends ServoMotorSubsystemWithCancoder {
    protected S mState;
    private final boolean allowAutoStateOutput;
    protected boolean atState = false;

    public StateBasedServoMotorSubsystemWithCancoder(ServoMotorSubsystemConstants constants, AbsoluteEncoderConstants encoderConstants, S initialState, boolean enableAutoStateOutput) {
        super(constants,encoderConstants);
        this.mState = initialState;
        this.allowAutoStateOutput = enableAutoStateOutput;
    }
    public StateBasedServoMotorSubsystemWithCancoder(ServoMotorSubsystemConstants constants, AbsoluteEncoderConstants encoderConstants,S initialState) {
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
        if (mState.isDisabled())
            super.setOpenLoop(0);
        super.writePeriodicOutputs();
    }

    @Override
    public void readPeriodicInputs() {
        super.readPeriodicInputs();
        atState = Util.epsilonEquals(getPosition(), demand, mState.getAllowableError());
        if(mState.isDisabled()||mControlState != ControlState.MOTION_MAGIC)
			atState = true;
    }

    @Override
    public void outputTelemetry() {
        Logger.recordOutput(mConstants.kName+"/AtState", atState);
		Logger.recordOutput(mConstants.kName+"/State",	mState);
        super.outputTelemetry();
    }

    /**
	 * Creates a request to change the state of the intake deployment.
	 *
	 * @param _wantedState The desired state.
	 * @return The request to change the state.
	 */
	public Request stateRequest(S _wantedState) {
		return new Request() {
			@Override
			public void act() {
				if (mControlState != ControlState.MOTION_MAGIC) {
					mControlState = ControlState.MOTION_MAGIC;
				}
				setState(_wantedState);
			}

			@Override
			public boolean isFinished() {
				
				return atState;
			}
		};
	}
}
