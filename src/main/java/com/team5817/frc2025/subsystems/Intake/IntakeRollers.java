package com.team5817.frc2025.subsystems.Intake;

import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.frc2025.subsystems.Intake.IntakeConstants.IntakeRollerConstants;
import com.team5817.lib.drivers.RollerSubsystem;
import com.team5817.lib.requests.Request;

import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class IntakeRollers extends RollerSubsystem {
	
	private static IntakeRollers mInstance;

	/**
	 * Gets the singleton instance of the IntakeRollers.
	 *
	 * @return The singleton instance.
	 */
	public static IntakeRollers getInstance() {
		if (mInstance == null) {
			mInstance = new IntakeRollers();
		}
		return mInstance;
	}

	public enum State {
		IDLE(0.0),
		INTAKING_CORAL(-10.0),
		INTAKING_ALGAE(8.0),
		SHOOTING_ALGAE(-8),
		EXHAUST(6.0);

		public double roller_voltage;

		State(double roller_voltage) {
			this.roller_voltage = roller_voltage;
		}
	}


	@Getter @Setter@Accessors(prefix = "m") private State mState = State.IDLE;

	/**
	 * Private constructor for the IntakeRollers subsystem.
	 */
	private IntakeRollers() {
		super(IntakeRollerConstants.kRollerConstants);
	}

	/**
	 * Registers the enabled loops for the subsystem.
	 *
	 * @param enabledLooper The looper to register.
	 */
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
			}

			@Override
			public void onLoop(double timestamp) {
				setVoltage(mState.roller_voltage);
			}
		});
	}

	/**
	 * Creates a new request to update the intake rollers with the wanted state.
	 *
	 * @param _wantedState Wanted state for the intake rollers.
	 * @return New request that updates the intake rollers with the wanted state.
	 */
	public Request stateRequest(State _wantedState) {
		return new Request() {
			@Override
			public void act() {
				setState(_wantedState);
			}

			@Override
			public boolean isFinished() {
				return true;
			}
		};
	}
}