package com.team5817.frc2024.subsystems;

import com.team5817.frc2024.subsystems.Climb.Climb;
import com.team5817.frc2024.subsystems.Elevator.Elevator;
import com.team5817.frc2024.subsystems.EndEffector.EndEffectorRollers;
import com.team5817.frc2024.subsystems.EndEffector.EndEffectorWrist;
import com.team5817.frc2024.subsystems.Indexer.Indexer;
import com.team5817.frc2024.subsystems.Intake.IntakeDeploy;
import com.team5817.frc2024.subsystems.Intake.IntakeRollers;

public class SuperstructureState {

	public final Elevator.State mElevatorState;
	public final EndEffectorWrist.State mEndEffectorWristState;
	public final IntakeDeploy.State mIntakeDeployState;
	public final Climb.State mClimbState;
	public final EndEffectorRollers.State mEndEffectorRollersState;
	public final IntakeRollers.State mIntakeRollersState;
	public final Indexer.State mIndexerState;
	public final Type mType;

	public enum Type{
		SCORING,
		INTAKING,
		CLIMBING
	};

	public SuperstructureState(
		Elevator.State elevator_state,
		EndEffectorWrist.State wrist_angle,
		IntakeDeploy.State intake_angle, 
		Climb.State climb_state, 
		EndEffectorRollers.State endEffector_state, 
		IntakeRollers.State intake_state, 
		Indexer.State indexer_state,
		Type type
		){

		this.mElevatorState = elevator_state;
		this.mEndEffectorWristState = wrist_angle;
		this.mIntakeDeployState = intake_angle;
		this.mClimbState = climb_state;
		this.mEndEffectorRollersState = endEffector_state;
		this.mIntakeRollersState = intake_state;
		this.mIndexerState = indexer_state;
		this.mType = type;
	}

}