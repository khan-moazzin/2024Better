package com.team5817.frc2025.subsystems.Intake;

import com.team5817.frc2025.subsystems.Intake.IntakeConstants.RollerConstants;
import com.team5817.lib.drivers.RollerSubsystemBasic.ControlState;
import com.team5817.lib.drivers.State.RollerState;
import com.team5817.lib.drivers.StateBasedRollerSubsystem;
import lombok.Getter;

public class IntakeRollers extends StateBasedRollerSubsystem<IntakeRollers.State> {

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

	public enum State implements RollerState {
		IDLE(0,0,0),
		INTAKING(10.0,2.5,8),
		HALF_INTAKING(10.0,0,0),
		EXHAUST(-6,-6,-6),
		IDLE_EXAUST(0,-2,-2);

		@Getter private double[] rollerDemands;
		@Getter private ControlState[] controlStates = {ControlState.VOLTAGE, ControlState.VOLTAGE, ControlState.VOLTAGE};

		State(double intakeVoltage, double indexerBottomVoltage, double indexerSidesVoltage) {
			this.rollerDemands = new double[] {intakeVoltage, indexerBottomVoltage, indexerSidesVoltage};
		}
	}

	/**
	 * Private constructor for the IntakeRollers subsystem.
	 */
	private IntakeRollers() {
		super(State.IDLE, RollerConstants.kIntakeConstants, RollerConstants.kIndexerBottomConstants, RollerConstants.kIndexerSideConstants);
	}
}