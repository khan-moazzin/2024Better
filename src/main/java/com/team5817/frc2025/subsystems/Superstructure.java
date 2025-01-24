package com.team5817.frc2025.subsystems;

import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.TimeDelayedBoolean;
import com.team5817.frc2025.Constants;
import com.team5817.frc2025.field.FieldLayout;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.frc2025.subsystems.Climb.Climb;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Elevator.Elevator;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorRollers;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorWrist;
import com.team5817.frc2025.subsystems.Indexer.Indexer;
import com.team5817.frc2025.subsystems.Intake.IntakeDeploy;
import com.team5817.frc2025.subsystems.Intake.IntakeRollers;
import com.team5817.lib.Lights.TimedLEDState;
import com.team5817.lib.drivers.BeamBreak;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;
import com.team5817.lib.requests.SequentialRequest;

import edu.wpi.first.math.util.Units;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

import javax.naming.PartialResultException;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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
	private boolean readyToScore = true;

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
		L1(new SuperstructureState(Elevator.State.L1, EndEffectorWrist.State.L1, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.CORAL_OUTTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.SCORING)),
		L2(new SuperstructureState(Elevator.State.L2, EndEffectorWrist.State.L2, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.CORAL_OUTTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.SCORING)),
		L3(new SuperstructureState(Elevator.State.L3, EndEffectorWrist.State.L3, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.CORAL_OUTTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.SCORING)),
		L4(new SuperstructureState(Elevator.State.L4, EndEffectorWrist.State.L4, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.CORAL_OUTTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.SCORING)),
		NET(new SuperstructureState(Elevator.State.NET, EndEffectorWrist.State.STOW, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.ALGAE_OUTTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.SCORING)),
		PROCESS(new SuperstructureState(Elevator.State.PROCESS, EndEffectorWrist.State.STOW, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.ALGAE_OUTTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.SCORING)),
		A1(new SuperstructureState(Elevator.State.A1, EndEffectorWrist.State.A1, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.ALGAE_INTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.CLEAN)), 
		A2(new SuperstructureState(Elevator.State.A2, EndEffectorWrist.State.A2, IntakeDeploy.State.STOW, Climb.State.STOW, EndEffectorRollers.State.ALGAE_INTAKE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.CLEAN)),
		GROUND_CORAL_INTAKE(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.INTAKING, IntakeDeploy.State.DEPLOY, Climb.State.STOW, EndEffectorRollers.State.CORAL_INTAKE, IntakeRollers.State.INTAKING_CORAL, Indexer.State.INDEXING, SuperstructureState.Type.INTAKING)),
		GROUND_ALGAE_INTAKE(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.STOW, IntakeDeploy.State.ALGAE, Climb.State.STOW, EndEffectorRollers.State.IDLE, IntakeRollers.State.INTAKING_ALGAE, Indexer.State.IDLE, SuperstructureState.Type.INTAKING)),
		HUMAN_CORAL_INTAKE(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.INTAKING, IntakeDeploy.State.HUMAN, Climb.State.STOW, EndEffectorRollers.State.CORAL_INTAKE, IntakeRollers.State.INTAKING_CORAL, Indexer.State.INDEXING, SuperstructureState.Type.INTAKING)),
		CLIMB_PREPARE(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.STOW, IntakeDeploy.State.ZERO, Climb.State.PREPARE, EndEffectorRollers.State.IDLE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.IDLE)),
		CLIMB_PULL(new SuperstructureState(Elevator.State.STOW, EndEffectorWrist.State.STOW, IntakeDeploy.State.ZERO, Climb.State.PULL, EndEffectorRollers.State.IDLE, IntakeRollers.State.IDLE, Indexer.State.IDLE, SuperstructureState.Type.IDLE));

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

	public void setActiveRequest(Request request) {
		activeRequest = request;
		hasNewRequest = true;
		allRequestsComplete = false;
	}

	public void clearRequestQueue() {
		queuedRequests.clear();
	}

	public void setRequestQueue(List<Request> requests) {
		clearRequestQueue();
		for (Request req : requests) {
			queuedRequests.add(req);
		}
	}

	public void setRequestQueue(Request activeRequest, ArrayList<Request> requests) {
		request(activeRequest);
		setRequestQueue(requests);
	}

	public void addRequestToQueue(Request req) {
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
		if(activeRequest!=null)
			Logger.recordOutput("State",activeRequest.getName());
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
		}.addName("AutoAlign Wait");
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
		).addName("Idle");
	}
	public Request GoalRequest(GoalState goal){
		
		switch (goal.goal.mType) {
			case SCORING:
				return ScoreGoalRequest(goal.goal);
			case IDLE:
			return null;//do climing last, wont affect anything unless you are climbing
			case INTAKING:
			return IntakeRequest(goal.goal);
			case CLEAN:
			return CleanRequest(goal.goal);
		}
		return null;
	}

	private Request CleanRequest(SuperstructureState goal){
		return new SequentialRequest(
			new ParallelRequest(
			mElevator.stateRequest(goal.mElevatorState),
			mIndexer.stateRequest(goal.mIndexerState),
			mIntakeDeploy.stateRequest(goal.mIntakeDeployState),
			mClimb.stateRequest(goal.mClimbState),
			mIntakeRollers.stateRequest(goal.mIntakeRollersState),
			mEndEffectorRollers.stateRequest(goal.mEndEffectorRollersState),
			mEndEffectorWrist.stateRequest(goal.mEndEffectorWristState)
			)
			// breakWait(null, true)
		).addName("Clean");
	}
	private Request IntakeRequest(SuperstructureState goal){
		if(!(goal.mType == SuperstructureState.Type.INTAKING)){
			System.out.println("Wrong Goal Type");
			return new ParallelRequest();
		}
		return new SequentialRequest(
			
			new ParallelRequest(
				mElevator.stateRequest(goal.mElevatorState),
				mEndEffectorWrist.stateRequest(goal.mEndEffectorWristState),
				mEndEffectorRollers.stateRequest(goal.mEndEffectorRollersState),
				mIndexer.stateRequest(goal.mIndexerState),
				mIntakeDeploy.stateRequest(goal.mIntakeDeployState),
				mClimb.stateRequest(goal.mClimbState),
				mIntakeRollers.stateRequest(goal.mIntakeRollersState)
						)
				// breakWait(null, true)
			).addName("Intake");
	}
	
	private Request ScoreGoalRequest(SuperstructureState goal) {
		if(!(goal.mType == SuperstructureState.Type.SCORING)){
			System.out.println("Wrong Goal Type");
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
			ReadyToScoreRequest(),
			mEndEffectorRollers.stateRequest(goal.mEndEffectorRollersState)
			// breakWait(null, false),//TODO
			
	).addName("Score");
	}


	private Request ReadyToScoreRequest() {
		return(new Request() {
			@Override
			public void act() {
			}

			@Override
			public boolean isFinished() {
				return readyToScore;
			}
		}).addName("Driver Score Wait");
	}
	public Request AlgaeSmartCleanRequest(){
		return isAlgaeHigh()?GoalRequest(GoalState.A2):GoalRequest(GoalState.A1);
	}
	private boolean isAlgaeHigh(){
		Translation2d reef_to_odom = FieldLayout.getReefPose().inverse().translateBy(mDrive.getPose().getTranslation());
		double angle = Math.atan2(reef_to_odom.x(),reef_to_odom.y());
		Logger.recordOutput("angle", reef_to_odom);
		angle = Units.radiansToDegrees(angle);
		angle += 30;
		int side = (int) Math.round(angle/60);
		Integer low = Math.floorMod(side, 2);
		return low==0;
	}


	/**
	 * Stop Intake Rollers, Serializer, Feeder, and Amp Rollers.
	 */
	public void idleState() {
		request(idleRequest());
	}
}