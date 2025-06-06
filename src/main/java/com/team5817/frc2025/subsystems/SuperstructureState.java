package com.team5817.frc2025.subsystems;

import com.team5817.frc2025.subsystems.Intake.Intake;
import com.team5817.frc2025.subsystems.Pivot.Pivot;
import com.team5817.frc2025.subsystems.Shooter.Shooter;

/**
 * Represents the state of the superstructure, including various subsystems.
 */
public class SuperstructureState {

	public final Pivot.State mPivotState;
	public final Shooter.State mShooterState;
	public final Intake.State mIntakeState;
	public final Type mType;
	
	/**
	 * Enum representing the type of superstructure state.
	 */
	public enum Type {
		INTAKING,
        SCORING,
        OFF,
        IDLE,
        AUTO,
        OUTTAKING
		
	};
	

	/**
	 * Constructs a SuperstructureState with the specified subsystem states, type, and alignment type.
	 *
	 * @param pivot_state the state of the pivot
	 * @param shooter_state the state of the end effector rollers
	 * @param intake_state the state of the indexer
	 * @param type the type of the superstructure state
	 */
	public SuperstructureState(
			Pivot.State pivot_state,
			Shooter.State shooter_state,
			Intake.State intake_state,
			Type type) {
	
		this.mPivotState = pivot_state;
		this.mShooterState = shooter_state;
		this.mIntakeState = intake_state;
		this.mType = type;

}
}