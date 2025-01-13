package com.team5817.frc2024.subsystems.Climb;

import com.team5817.lib.Util;
import com.team5817.lib.drivers.ServoMotorSubsystem;
import com.team5817.lib.requests.Request;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2024.Constants.ClimbConstants;
import com.team5817.frc2024.loops.ILooper;
import com.team5817.frc2024.loops.Loop;

public class Climb extends ServoMotorSubsystem {
	public static Climb mInstance;

	public static Climb getInstance() {
		if (mInstance == null) {
			mInstance = new Climb(ClimbConstants.kClimbServoConstants);
		}
		return mInstance;
	}

	final static double kStrictError = .5;
	final static double kMediumError = 2;
	final static double kLenientError = 5;

    public enum State {
        ZERO(0.0, kStrictError),
        STOW(0.0, kLenientError),
        PULL(0.0, kStrictError),
		PREPARE(0.0, kLenientError);

        double output = 0;
		double allowable_error = 0;

        State(double output, double allowable_error) {
        this.output = output;
		this.allowable_error = allowable_error;
        }
    }


	public Climb(final ServoMotorSubsystemConstants constants) {
		super(constants);
		mMain.setPosition(homeAwareUnitsToRotations(120.0));
		enableSoftLimits(false);
		setSetpointMotionMagic(State.ZERO.output);
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
        Logger.processInputs("Climb", mServoInputs);
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