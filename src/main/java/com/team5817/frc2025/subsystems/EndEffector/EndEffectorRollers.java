package com.team5817.frc2025.subsystems.EndEffector;

import com.team5817.frc2025.subsystems.EndEffector.EndEffectorConstants.RollerConstants;
import com.team5817.lib.drivers.BasicStateBasedRollerSubsystem;
import com.team5817.lib.drivers.State.BasicRollerState;

import lombok.Getter;

/**
 * The EndEffectorRollers subsystem controls the rollers of the end effector.
 */
public class EndEffectorRollers extends BasicStateBasedRollerSubsystem<EndEffectorRollers.State> {
	private static EndEffectorRollers mInstance;

	/**
	 * Gets the singleton instance of the EndEffectorRollers subsystem.
	 *
	 * @return The singleton instance.
	 */
	public static EndEffectorRollers getInstance() {
		if (mInstance == null) {
			mInstance = new EndEffectorRollers();
		}
		return mInstance;
	}
	
	public enum State implements BasicRollerState {
		IDLE(0.0),
		HOLD(-2.0),
		HOLDCORAL(-1.5),
		CORAL_INTAKE(-4.0),
		l4(12.0),
		l3(3.0),
		l2(8.0),
		l1(1.5),
		ALGAE_INTAKE(-9.0),
		ALGAE_OUTTAKE(1.25),
		ALGAE_SHOOT(12);

		@Getter private double rollerDemand;

		State(double roller_voltage) {
			this.rollerDemand = roller_voltage;
		}

		@Override
		public ControlState getControlState() {
			return ControlState.VOLTAGE;
		}
	}

	/**
	 * Private constructor for the EndEffectorRollers subsystem.
	 */
	private EndEffectorRollers() {
		super(State.IDLE, RollerConstants.kEndEffectorRollerConstants);
	}
}