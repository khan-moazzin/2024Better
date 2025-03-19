package com.team5817.frc2025.subsystems;

import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.field.FieldConstants.ReefLevel;
import com.team5817.frc2025.subsystems.Climb.Climb;
import com.team5817.frc2025.subsystems.Elevator.Elevator;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorRollers;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorWrist;
import com.team5817.frc2025.subsystems.Indexer.Indexer;
import com.team5817.frc2025.subsystems.Intake.IntakeDeploy;
import com.team5817.frc2025.subsystems.Intake.IntakeRollers;

/**
 * Represents the state of the superstructure, including various subsystems.
 */
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
	
	/**
	 * Enum representing the type of superstructure state.
	 */
	public enum Type {
		SCORING,
		CLEAN,
		INTAKING,
		IDLE,
		NET,
		SMARTCORALINTAKE;
		
	};
	
	/**
	 * Constructs a SuperstructureState with the specified subsystem states and type.
	 *
	 * @param elevator_state the state of the elevator
	 * @param wrist_state the state of the end effector wrist
	 * @param intake_state the state of the intake deploy
	 * @param climb_state the state of the climb
	 * @param endEffector_state the state of the end effector rollers
	 * @param intake_roller_state the state of the intake rollers
	 * @param indexer_state the state of the indexer
	 * @param type the type of the superstructure state
	 */
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

	/**
	 * Constructs a SuperstructureState with the specified subsystem states, type, and alignment type.
	 *
	 * @param elevator_state the state of the elevator
	 * @param wrist_state the state of the end effector wrist
	 * @param intake_state the state of the intake deploy
	 * @param climb_state the state of the climb
	 * @param endEffector_state the state of the end effector rollers
	 * @param intake_roller_state the state of the intake rollers
	 * @param indexer_state the state of the indexer
	 * @param type the type of the superstructure state
	 * @param alignmentTypes the alignment type
	 */
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
	/**
	 * Constructs a SuperstructureState with the specified subsystem states, type, and alignment type.
	 *
	 * @param elevator_state the state of the elevator
	 * @param wrist_state the state of the end effector wrist
	 * @param intake_state the state of the intake deploy
	 * @param climb_state the state of the climb
	 * @param endEffector_state the state of the end effector rollers
	 * @param intake_roller_state the state of the intake rollers
	 * @param indexer_state the state of the indexer
	 * @param type the type of the superstructure state
	 * @param alignmentTypes the alignment type
	 */
	public SuperstructureState(
			Elevator.State elevator_state,
			EndEffectorWrist.State wrist_state,
			IntakeDeploy.State intake_state,
			Climb.State climb_state,
			EndEffectorRollers.State endEffector_state,
			IntakeRollers.State intake_roller_state,
			Indexer.State indexer_state,
			Type type, AlignmentType alignmentTypes,ReefLevel level) {
	
		this.mElevatorState = elevator_state;
		this.mEndEffectorWristState = wrist_state;
		this.mIntakeDeployState = intake_state;
		this.mClimbState = climb_state;
		this.mEndEffectorRollersState = endEffector_state;
		this.mIntakeRollersState = intake_roller_state;
		this.mIndexerState = indexer_state;
		this.mType = type;
		this.mAlignmentType = alignmentTypes;
		this.level = level;
}
		ReefLevel level = ReefLevel.L4;
}