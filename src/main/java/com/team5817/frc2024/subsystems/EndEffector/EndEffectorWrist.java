package com.team5817.frc2024.subsystems.EndEffector;

import com.team5817.lib.Util;
import com.team5817.lib.drivers.ServoMotorSubsystemWithCancoder;
import com.team5817.lib.requests.Request;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2024.Constants.EndEffectorWristConstants;
import com.team5817.frc2024.loops.ILooper;
import com.team5817.frc2024.loops.Loop;
public class EndEffectorWrist extends ServoMotorSubsystemWithCancoder {
	public static EndEffectorWrist mInstance;

	public static EndEffectorWrist getInstance() {
		if (mInstance == null) {
			mInstance = new EndEffectorWrist(EndEffectorWristConstants.kWristServoConstants, EndEffectorWristConstants.kWristEncoderConstants);
		}
		return mInstance;
	}

	final static double kStrictError = .5;
	final static double kMediumError = 2;
	final static double kLenientError = 5;


    public enum State {

		L4(0.0, kStrictError),
        L3(0.0, kStrictError),
        L2(0.0, kStrictError),
        L1(0.0, kStrictError),
        A1(0.0, kMediumError),
        A2(0.0, kMediumError),
        NET(0.0, kMediumError),
        ZERO(0.0, kLenientError),
		PROCESS(0.0, kLenientError),
        INTAKING(0.0, kStrictError),
        STOW(0.0, kStrictError);


        double output = 0;
		double allowable_error = 20;

        State(double output, double allowable_error) {
            this.output = output;
			this.allowable_error = allowable_error;
        }
    }



	public EndEffectorWrist(final ServoMotorSubsystemConstants constants, final AbsoluteEncoderConstants encoder_constants) {
		super(constants, encoder_constants);
		mMain.setPosition(homeAwareUnitsToRotations(120.0));
		enableSoftLimits(false);
		setSetpointMotionMagic(State.STOW.output);
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
			}

			@Override
			public void onStop(double timestamp) {
				setNeutralMode(NeutralModeValue.Brake);
			}
		});
	}

    @Override
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();
		
        Logger.processInputs("EndEffectorWrist", mServoInputs);
    }

	@Override
	public synchronized void writePeriodicOutputs() {
		super.writePeriodicOutputs();
	}

	@Override
	public void outputTelemetry() {
		super.outputTelemetry();
	}

	@Override
	public void stop() {}

	@Override
	public boolean checkSystem() {
		return false;
	}

	public Request stowRequest() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.STOW.output);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.STOW.output, State.STOW.allowable_error);
			}
		};
	}

	public Request zeroRequest() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.ZERO.output);
			}
			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.ZERO.output, State.ZERO.allowable_error);
			}
		};
	}

	public Request stateRequest(State _wantedState) {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(_wantedState.output);
			}
			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), _wantedState.output, _wantedState.allowable_error);
			}
		};
	}

}