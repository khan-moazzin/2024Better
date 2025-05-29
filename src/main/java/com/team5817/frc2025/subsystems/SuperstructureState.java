package com.team5817.frc2025.subsystems;

import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.subsystems.Intake.Intake;
import com.team5817.frc2025.subsystems.Pivot.Pivot;
import com.team5817.frc2025.subsystems.Shooter.Shooter;

/**
 * Represents the state of the superstructure, including various subsystems.
 */
public class SuperstructureState {

	public final Pivot.State mPivotState;
	public final Shooter.State mEndEffectorRollersState;
	public final Intake.State mIntakeState;
	public final Type mType;
	public final AlignmentType mAlignmentType;
	
	/**
	 * Enum representing the type of superstructure state.
	 */
	public enum Type {
		INTAKING,
        SCORE,
        OFF,
        IDLE,
        AUTO,
        OUTTAKING
		
	};
	

	/**
	 * Constructs a SuperstructureState with the specified subsystem states, type, and alignment type.
	 *
	 * @param pivot_state the state of the pivot
	 * @param endEffector_state the state of the end effector rollers
	 * @param indexer_state the state of the indexer
	 * @param type the type of the superstructure state
	 * @param alignmentTypes the alignment type
	 */
	public SuperstructureState(
			Pivot.State pivot_state,
			Shooter.State endEffector_state,
			Intake.State intake_state,
			Type type, AlignmentType alignmentTypes) {
	
		this.mPivotState = pivot_state;
		this.mEndEffectorRollersState = endEffector_state;
		this.mIntakeState = intake_state;
		this.mType = type;
		this.mAlignmentType = alignmentTypes;

}
}