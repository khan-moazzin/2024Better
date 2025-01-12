package com.team5817.frc2024.subsystems;

import com.team254.lib.util.TimeDelayedBoolean;
import com.team5817.frc2024.Constants;
import com.team5817.frc2024.loops.ILooper;
import com.team5817.frc2024.loops.Loop;
import com.team5817.frc2024.subsystems.Drive.Drive;
import com.team5817.frc2024.subsystems.Elevator.Elevator;
import com.team5817.frc2024.subsystems.SuperstructureState;
import com.team5817.frc2024.subsystems.Climb.Climb;
import com.team5817.frc2024.subsystems.EndEffector.EndEffectorWrist;
import com.team5817.frc2024.subsystems.EndEffector.EndEffectorRollers;
import com.team5817.frc2024.subsystems.EndEffector.EndEffectorRollers.State;
import com.team5817.frc2024.subsystems.Indexer.Indexer;
import com.team5817.frc2024.subsystems.Intake.IntakeDeploy;
import com.team5817.frc2024.subsystems.Intake.IntakeRollers;
import com.team5817.lib.Lights.TimedLEDState;
import com.team5817.lib.drivers.BeamBreak;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.LambdaRequest;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;
import com.team5817.lib.requests.SequentialRequest;

import java.util.ArrayList;
import java.util.List;

public class Superstructure extends Subsystem {

	private static Superstructure mInstance;

	public static synchronized Superstructure getInstance() {
		if (mInstance == null) {
			mInstance = new Superstructure();
		}

		return mInstance;
	}

	// Request tracking variables
	private Request activeRequest = null;
	private ArrayList<Request> queuedRequests = new ArrayList<>(0);
	private boolean hasNewRequest = false;
	private boolean allRequestsComplete = false;
	private boolean readyToScore = false;

	// Subsystems

	// LEDs
	private final LEDs mLEDs = LEDs.getInstance();
	private TimedLEDState mHeldState = TimedLEDState.NOTE_HELD_SHOT;


	// Target tracking
	private Drive mDrive;
	private double mDistanceToTarget = 0.0;
	private double mAngularErrToTarget = 0.0;
	private SuperstructureState mGoal;


	private Elevator mElevator;
	private EndEffectorWrist mEndEffectorWrist;
	private IntakeDeploy mIntakeDeploy;
	private Climb mClimb;
	private EndEffectorRollers mEndEffectorRollers;
	private IntakeRollers mIntakeRollers;
	private Indexer mIndexer;

	public enum GameObject{
		CORAL,
		ALGAE
	}

	public enum GoalState{
		STOW(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.STOW, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.IDLE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.IDLE)),
		L1(new SuperstructureState(Elevator.State.L1, EndEffectorWrist.State.L1, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.CORAL_OUTTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.REEF)),
		L2(new SuperstructureState(Elevator.State.L2, EndEffectorWrist.State.L2, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.CORAL_OUTTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.REEF)),
		L3(new SuperstructureState(Elevator.State.L3, EndEffectorWrist.State.L3, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.CORAL_OUTTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.REEF)),
		L4(new SuperstructureState(Elevator.State.L4, EndEffectorWrist.State.L4, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.CORAL_OUTTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.REEF)),
		NET(new SuperstructureState(Elevator.State.NET, EndEffectorWrist.State.STOW, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.ALGAE_OUTTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.REEF)),
		PROCESS(new SuperstructureState(Elevator.State.PROCESS, EndEffectorWrist.State.STOW, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.ALGAE_OUTTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.REEF)),
		A1(new SuperstructureState(Elevator.State.A1, EndEffectorWrist.State.A1, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.ALGAE_INTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.REEF)), //TODO THESE ARE LABLED AS SCORING BUT ARE ACTUALLY INTAKING
		A2(new SuperstructureState(Elevator.State.A2, EndEffectorWrist.State.A2, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.ALGAE_INTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.REEF)),
		GROUND_CORAL_INTAKE(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.INTAKING, IntakeDeploy.State.DEPLOY, Climb.State.STOW, EndEffectorRollers.State.CORAL_INTAKE, IntakeRollers.State.INTAKING_CORAL, Indexer.State.INDEXING, SuperstructureState.Type.INTAKING)),
		GROUND_ALGAE_INTAKE(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.STOW, IntakeDeploy.State.ALGAE, Climb.State.STOW, EndEffectorRollers.State.IDLE, IntakeRollers.State.INTAKING_ALGAE, Indexer.State.IDLE, SuperstructureState.Type.INTAKING)),
		HUMAN_CORAL_INTAKE(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.INTAKING, IntakeDeploy.State.HUMAN, Climb.State.STOW, EndEffectorRollers.State.CORAL_INTAKE, IntakeRollers.State.INTAKING_CORAL, Indexer.State.INDEXING, SuperstructureState.Type.INTAKING)),
		CLIMB_PREPARE(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.STOW, IntakeDeploy.State.ZERO, Climb.State.PREPARE, EndEffectorRollers.State.IDLE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.CLIMBING)),
		CLIMB_PULL(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.STOW, IntakeDeploy.State.ZERO, Climb.State.PULL, EndEffectorRollers.State.IDLE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.CLIMBING));

		SuperstructureState goal;
		GoalState(SuperstructureState state_goal){
			this.goal = state_goal;
		}
	}

	Superstructure() {
		mDrive = Drive.getInstance();
		mElevator = Elevator.getInstance();
		mEndEffectorWrist = EndEffectorWrist.getInstance();
		mIntakeDeploy = IntakeDeploy.getInstance();
		mClimb = Climb.getInstance();
		mEndEffectorRollers = EndEffectorRollers.getInstance();
		mIntakeRollers = IntakeRollers.getInstance();
		mIndexer = Indexer.getInstance();
	}
	public boolean requestsCompleted() {
		return allRequestsComplete;
	}

	public void request(Request r) {
		setActiveRequest(r);
		clearRequestQueue();
	}

	private void setActiveRequest(Request request) {
		activeRequest = request;
		hasNewRequest = true;
		allRequestsComplete = false;
	}

	private void clearRequestQueue() {
		queuedRequests.clear();
	}

	private void setRequestQueue(List<Request> requests) {
		clearRequestQueue();
		for (Request req : requests) {
			queuedRequests.add(req);
		}
	}

	private void setRequestQueue(Request activeRequest, ArrayList<Request> requests) {
		request(activeRequest);
		setRequestQueue(requests);
	}

	private void addRequestToQueue(Request req) {
		queuedRequests.add(req);
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				clearRequestQueue();
			}

			@Override
			public void onLoop(double timestamp) {
				try {
					if (hasNewRequest && activeRequest != null) {
						activeRequest.act();
						hasNewRequest = false;
					}

					if (activeRequest == null) {
						if (queuedRequests.isEmpty()) {
							allRequestsComplete = true;
						} else {
							request(queuedRequests.remove(0));
						}
					} else if (activeRequest.isFinished()) {
						activeRequest = null;
					}

				} catch (Exception e) {
					e.printStackTrace();
				}
			}

			@Override
			public void onStop(double timestamp) {
				stop();
			}
		});
	}

	@Override
	public void stop() {
		activeRequest = null;
		clearRequestQueue();
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void readPeriodicInputs() {
	}

	@Override
	public void outputTelemetry() {
	}

	/* Superstructure functions */



	
	/**
	 * BeamBreak Sensor reading.
	 * 
	 * @param mBreak BeamBreak Sensor.
	 * @param target_state If wanted reading is true (broken) or false (not broken).
	 * 
	 * @return Boolean for if target state is acheived. 
	 */
	private Request breakWait(BeamBreak mBreak, boolean target_state) {
		return new Request() {

			@Override
			public void act() {}

			@Override
			public boolean isFinished() {
				return mBreak.get() == target_state;
			}
		};
	}

	/**
	 * Debounced BeamBreak Sensor reading.
	 * 
	 * @param mBreak BeamBreak Sensor.
	 * @param target_state If wanted reading is true (broken) or false (not broken).
	 * @param delayed_wait_seconds Debounces time from a BeamBreak Sensor. 
	 * 
	 * @return Boolean for if target state is acheived after debouncing the signal. 
	 */
	private Request breakWait(BeamBreak mBreak, boolean target_state, double delayed_wait_seconds) {
		return new Request() {

			TimeDelayedBoolean timeout = new TimeDelayedBoolean();

			@Override
			public void act() {}

			@Override
			public boolean isFinished() {
				return timeout.update(mBreak.get() == target_state, delayed_wait_seconds);
			}
		};
	}

	private Request autoAlignWait() {
		return new Request() {
			@Override
			public void act() {
			}

			@Override
			public boolean isFinished() {
				return mDrive.getAutoAlignComplete();
			}
		};
	}

	/**
	 * Update state of LEDs based on BeamBreak readings.
	 */
	private Request updateLEDsRequest() {
		return new Request() {

			@Override
			public void act() {
				updateLEDs();
			}

			@Override
			public boolean isFinished() {
				return true;
			}
		};
	}

	/**
	 * Update state of LEDs based on BeamBreak readings.
	 */
	private void updateLEDs() {
	}


	/**
	 * Stop Intake Rollers, Serializer, Feeder, and Amp Rollers.
	 */
	private Request idleRequest() {
		return new ParallelRequest(
		);
	}

	private Request reefGoalRequest(SuperstructureState goal) {
		if(!(goal.mType == SuperstructureState.Type.REEF)){
			return new ParallelRequest(
			);
		}
		return new SequentialRequest(
			new ParallelRequest(
			mElevator.stateRequest(goal.mElevatorState),
			mIndexer.stateRequest(goal.mIndexerState),
			mIntakeDeploy.stateRequest(goal.mIntakeDeployState),
			mClimb.stateRequest(goal.mClimbState),
			mIntakeRollers.stateRequest(goal.mIntakeRollersState),
			new SequentialRequest(
				mElevator.waitForExtensionRequest(Constants.ElevatorConstants.kCoralClearHeight),
				mEndEffectorWrist.stateRequest(goal.mEndEffectorWristState)
			)
			),
			autoAlignWait(),
			readyToScoreRequest(),
			mEndEffectorRollers.stateRequest(goal.mEndEffectorRollersState),
			breakWait(null, false),
			new ParallelRequest(
				mIntakeDeploy.stowRequest(),
				mEndEffectorWrist.stowRequest(),
				mElevator.zeroRequest(),
				mEndEffectorRollers.idleRequest()
			)
	);
	}

	public Request readyToScoreRequest() {
		return(new Request() {
			@Override
			public void act() {
			}

			@Override
			public boolean isFinished() {
				return readyToScore;
			}
		});
	}


	/**
	 * Stop Intake Rollers, Serializer, Feeder, and Amp Rollers.
	 */
	public void idleState() {
		request(idleRequest());
	}
}