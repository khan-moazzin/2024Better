package com.team5817.frc2025.subsystems;

import java.util.List;

import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.subsystems.Climb.Climb;
import com.team5817.frc2025.subsystems.Elevator.Elevator;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorRollers;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorWrist;
import com.team5817.frc2025.subsystems.Indexer.Indexer;
import com.team5817.frc2025.subsystems.Intake.IntakeDeploy;
import com.team5817.frc2025.subsystems.Intake.IntakeRollers;

public class SuperstructureState {

	public final Elevator.State mElevatorState;
	public final EndEffectorWrist.State mEndEffectorWristState;
	public final IntakeDeploy.State mIntakeDeployState;
	public final Climb.State mClimbState;
	public final EndEffectorRollers.State mEndEffectorRollersState;
	public final IntakeRollers.State mIntakeRollersState;
	public final Indexer.State mIndexerState;
	public final Type mType;
	public final AlignmentType mAlignmentType;
	
		public enum Type {
			SCORING,
			CLEAN,
			INTAKING,
			IDLE
		};
	
		public SuperstructureState(
				Elevator.State elevator_state,
				EndEffectorWrist.State wrist_state,
				IntakeDeploy.State intake_state,
				Climb.State climb_state,
				EndEffectorRollers.State endEffector_state,
				IntakeRollers.State intake_roller_state,
				Indexer.State indexer_state,
				Type type) {
	
			this.mElevatorState = elevator_state;
			this.mEndEffectorWristState = wrist_state;
			this.mIntakeDeployState = intake_state;
			this.mClimbState = climb_state;
			this.mEndEffectorRollersState = endEffector_state;
			this.mIntakeRollersState = intake_roller_state;
			this.mIndexerState = indexer_state;
			this.mType = type;
			this.mAlignmentType = AlignmentType.NONE;
		}
		public SuperstructureState(
			Elevator.State elevator_state,
			EndEffectorWrist.State wrist_state,
			IntakeDeploy.State intake_state,
			Climb.State climb_state,
			EndEffectorRollers.State endEffector_state,
			IntakeRollers.State intake_roller_state,
			Indexer.State indexer_state,
			Type type, AlignmentType alignmentTypes) {
	
		this.mElevatorState = elevator_state;
		this.mEndEffectorWristState = wrist_state;
		this.mIntakeDeployState = intake_state;
		this.mClimbState = climb_state;
		this.mEndEffectorRollersState = endEffector_state;
		this.mIntakeRollersState = intake_roller_state;
		this.mIndexerState = indexer_state;
		this.mType = type;
		this.mAlignmentType = alignmentTypes;
}

}