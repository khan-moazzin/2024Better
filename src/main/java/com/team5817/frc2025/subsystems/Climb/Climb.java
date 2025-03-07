package com.team5817.frc2025.subsystems.Climb;

import com.team5817.frc2025.Constants.ClimbConstants;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.ServoMotorSubsystem;
import com.team5817.lib.requests.Request;

import org.littletonrobotics.junction.Logger;

/**
 * The Climb subsystem controls the climbing mechanism of the robot.
 */
public class Climb extends ServoMotorSubsystem {
	public static Climb mInstance;

	/**
	 * Returns the singleton instance of the Climb subsystem.
	 * @return The Climb instance.
	 */
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

        /**
         * Constructs a State with the specified output and allowable error.
         * @param output The output position.
         * @param allowable_error The allowable error margin.
         */
        State(double output, double allowable_error) {
        this.output = output;
		this.allowable_error = allowable_error;
        }
    }


	/**
	 * Constructs the Climb subsystem with the specified constants.
	 * @param constants The constants for the servo motor subsystem.
	 */
	public Climb(final ServoMotorSubsystemConstants constants) {
		super(constants);
		mMain.setPosition(homeAwareUnitsToRotations(120.0));
		enableSoftLimits(false);
		setSetpointMotionMagic(State.ZERO.output);
	}

	/**
	 * Registers the enabled loops for the Climb subsystem.
	 * @param enabledLooper The enabled looper.
	 */
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
		}
		});
	}

    @Override
    public  void readPeriodicInputs() {
        super.readPeriodicInputs();
        Logger.processInputs("Climb", mServoInputs);
    }

	@Override
	public  void writePeriodicOutputs() {
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

	/**
	 * Creates a request to change the state of the Climb subsystem.
	 * @param _wantedState The desired state.
	 * @return The request to change the state.
	 */
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