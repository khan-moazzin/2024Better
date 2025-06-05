package com.team5817.frc2025.subsystems;

import java.util.Map;


import com.google.flatbuffers.Constants;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

import com.team5817.frc2025.RobotState;
import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.field.FieldConstants;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Pivot.Pivot;
import com.team5817.frc2025.subsystems.Pivot.PivotConstants;
import com.team5817.frc2025.subsystems.Shooter.Shooter;
import com.team5817.frc2025.subsystems.Shooter.ShooterConstants;
import com.team5817.frc2025.subsystems.Intake.Intake; 
import com.team5817.frc2025.subsystems.Intake.IntakeRollers; 
import com.team5817.frc2025.subsystems.Intake.IndexerRollers; 
import com.team5817.frc2025.subsystems.Intake.IntakeConstants; 
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;
import com.team5817.lib.requests.SequentialRequest;
import com.team5817.lib.requests.WaitRequest;
import com.team5817.lib.util.ShootingUtils;
import com.team5817.lib.util.ShootingUtils.ShootingParameters;
import com.team5817.lib.util.ShootingParametersBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;



public class Superstructure extends Subsystem {

	private static Superstructure mInstance;

	/**
	 * Returns the singleton instance of the Superstructure.
	 * 
	 * @return The singleton instance of the Superstructure.
	 */
	public static Superstructure getInstance() {
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
	private boolean driverAllowsPoseComp = true;
	private double pivotOffset = 0;

	private GoalState mGoal= GoalState.STOW;
    protected RobotState mRobotState;

	// Subsystems
	public Drive mDrive;
	public Pivot mPivot;
	public Shooter mShooter;
	public Intake mIntake;
//SUPERSTRUCTURE STATE FIXED WITH CORRECT ARGUMENTS, WORK ON SUPERSTSRUCTURE


	public enum Mode {
		IDLE,
		INTAKING,
		OUTTAKING,
		SCORING,
		FIRING,
		SHOOTING,
    }
	
	
	public enum GoalState {
			
		ZERO(new SuperstructureState(
				Pivot.State.ZERO, 
				Shooter.State.IDLE, 
				Intake.State.IDLE, 
				SuperstructureState.Type.IDLE)),

		IDLE(new SuperstructureState(
				Pivot.State.STOW, 
				Shooter.State.IDLE, 
				Intake.State.IDLE, 
				SuperstructureState.Type.IDLE)),
		
		STOW(new SuperstructureState(
		    Pivot.State.STOW, 
			Shooter.State.IDLE, 
			Intake.State.IDLE, 
			SuperstructureState.Type.IDLE)),
		
		INTAKE(new SuperstructureState(
			Pivot.State.TRANSFER, 
			Shooter.State.TRANSFER, 
			Intake.State.INTAKING, 
			SuperstructureState.Type.IDLE)),

		OUTTAKE(new SuperstructureState(
			Pivot.State.CLEAR, 
			Shooter.State.REVERSETRANSFER, 
			Intake.State.OUTTAKING, 
			SuperstructureState.Type.IDLE)),
				
		SHOOTING(new SuperstructureState(
				Pivot.State.SHOOTING,
				Shooter.State.SHOOTING, 
				Intake.State.HALF_INTAKING, 
				SuperstructureState.Type.SCORING)),

		FIRING(new SuperstructureState(
				Pivot.State.SHOOTING, 
				Shooter.State.SHOOTING, 
				Intake.State.IDLE, 
				SuperstructureState.Type.SCORING));

		public SuperstructureState goal;

		GoalState(SuperstructureState state_goal) {
			this.goal = state_goal;
		}
	}
	
	/**
	 * Constructor for the Superstructure class.
	 */
	Superstructure() {
		mDrive = Drive.getInstance();
		mPivot = Pivot.getInstance();
		mShooter = Shooter.getInstance();
		mIntake = Intake.getInstance();
	}

	/**
	 * Checks if all requests have been completed.
	 * 
	 * @return True if all requests are complete, false otherwise.
	 */
	public boolean requestsCompleted() {
		return allRequestsComplete;
	}

	/**
	 * Sets a new active request and clears the request queue.
	 * 
	 * @param r The new request to be set as active.
	 */
	public void request(Request r) {
		setActiveRequest(r);
		clearRequestQueue();
	}

	/**
	 * Sets the active request.
	 * 
	 * @param request The request to be set as active.
	 */
	public void setActiveRequest(Request request) {
		activeRequest = request;
		hasNewRequest = true;
		allRequestsComplete = false;
	}

	/**
	 * Clears the request queue.
	 */
	public void clearRequestQueue() {
		queuedRequests.clear();
	}

	/**
	 * Sets the request queue with a list of requests.
	 * 
	 * @param requests The list of requests to be added to the queue.
	 */
	public void setRequestQueue(List<Request> requests) {
		clearRequestQueue();
		for (Request req : requests) {
			queuedRequests.add(req);
		}
	}

	/**
	 * Sets the request queue with an active request and a list of requests.
	 * 
	 * @param activeRequest The active request to be set.
	 * @param requests      The list of requests to be added to the queue.
	 */
	public void setRequestQueue(Request activeRequest, ArrayList<Request> requests) {
		request(activeRequest);
		setRequestQueue(requests);
	}

	/**
	 * Adds a request to the queue.
	 * 
	 * @param req The request to be added to the queue.
	 */
	public void addRequestToQueue(Request req) {
		queuedRequests.add(req);
	}


		public boolean inShootZone(double timestamp) {
        Pose2d currentPose = mRobotState.getGlobalKalmanPose(timestamp);
        if (DriverStation.getAlliance().get().equals(Alliance.Red))
            return currentPose.getTranslation().x() > 8.25;
        return currentPose.getTranslation().x() < 8.25;
    }
	


public ShootingParameters getShootingParams(Pose2d currentPose) {
    Pose2d speakerPose = FieldConstants.getSpeakerPivotPose();

    return new ShootingParametersBuilder()
            .setCurrentPose(currentPose)
            .setTargetPose(speakerPose)
            .setPivotMap(ShootingUtils.getPivotMap(false)) 
            .setVelocityMap(ShootingUtils.getVelocityMap(false)) 
            .setCurrentVelocity(mRobotState.getPredictedVelocity())
            .setManual(false)
            .build();
}

public ShootingParameters getShootingParams(Pose2d currentPose, boolean manual, boolean lob) {
    Pose2d speakerPose = FieldConstants.getSpeakerPivotPose();

    return new ShootingParametersBuilder()
            .setPivotOffset(pivotOffset)
            .setCurrentPose(currentPose)
            .setTargetPose(speakerPose)
            .setPivotMap(ShootingUtils.getPivotMap(lob)) 
            .setVelocityMap(ShootingUtils.getVelocityMap(lob)) 
            .setCurrentVelocity(mRobotState.getPredictedVelocity())
            .setManual(manual)
            .build();
}



public void prepareShooterSetpoints(double timestamp) {
	ShootingParameters shootingParameters = getShootingParams(mRobotState.getKalmanPose(timestamp));
	Shooter.State state = Shooter.State.SHOOTING;
	mShooter.setVelocitySetpoint(shootingParameters.compensatedDesiredShooterSpeed);
	mShooter.setSpin(shootingParameters.spin);

	mShooter.conformToState(Shooter.State.SHOOTING);
	mPivot.conformToState(shootingParameters.compensatedDesiredPivotAngle - 1);
}

	
public void prepareShooterSetpoints(double timestamp, boolean manual) {
	ShootingParameters shootingParameters = getShootingParams(mRobotState.getKalmanPose(timestamp), manual,
			!inShootZone(timestamp));
	Shooter.State state = Shooter.State.SHOOTING;
	if (inShootZone(timestamp)) {
		mShooter.conformToState(state);
		mShooter.setSpin(shootingParameters.spin);
	} else {
		mShooter.setVelocitySetpoint(shootingParameters.compensatedDesiredShooterSpeed);
		mShooter.setSpin(1);
	}
	mPivot.conformToState(shootingParameters.compensatedDesiredPivotAngle);
}




	
	public void manageRequests(){
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
	public void stop() {
		activeRequest = null;
		clearRequestQueue();
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	/**
	 * Gets the current goal state.
	 * 
	 * @return The current goal state.
	 */
	public GoalState getGoalState() {
		return mGoal;
	}

	@Override
	public void outputTelemetry() {
		if (activeRequest != null)
			Logger.recordOutput("State", activeRequest.getName());
		Logger.recordOutput("Elastic/AllowedPoseComp", driverAllowsPoseComp);
	}

	/* Superstructure functions */

	/**
	 * Sets the goal state and creates a corresponding request.
	 * 
	 * @param goal The goal state to be set.
	 */
	public void setGoal(GoalState goal) {
		Request r = new ParallelRequest();
		if (mGoal != goal) {
			mGoal = goal;
			switch (goal.goal.mType) {
				case IDLE:
					r = IdleRequest(goal.goal);
					break;
				case SCORING:
					r = ScoreGoalRequest(goal.goal);
					break;
				case INTAKING:
					r = IntakeRequest(goal.goal);
					break;
				case OUTTAKING:
					r = CleanRequest(goal.goal);
					break;
			}
			request(r);
		}
		Logger.recordOutput("Goal", goal);
	}

	/**
	 * Creates a request for cleaning based on the goal state.
	 * 
	 * @param goal The goal state.
	 * @return A request for cleaning.
	 */
	private Request CleanRequest(SuperstructureState goal) {
		return new SequentialRequest(
				new ParallelRequest(
						mPivot.stateRequest(goal.mPivotState),
						mIntake.stateRequest(goal.mIntakeState),
						mShooter.stateRequest(goal.mShooterState)
						)
				).addName("Clean");
	}

	private Request IdleRequest(SuperstructureState goal) {
		if (!(goal.mType == SuperstructureState.Type.IDLE)) {
			System.out.println("Wrong Goal Type");
			return new ParallelRequest();
		}
		return new SequentialRequest(
			new ParallelRequest(
				new SequentialRequest(
					new WaitRequest(0.1),
					mPivot.stateRequest(goal.mPivotState)),
				mShooter.stateRequest(Shooter.State.IDLE),
				mIntake.stateRequest(goal.mIntakeState)),
				mShooter.stateRequest(goal.mShooterState)
						
		).addName("Idle");
	}


	/**
	 * Creates a request for intaking based on the goal state.
	 * 
	 * @param goal The goal state.
	 * @return A request for intaking.
	 */
	private Request IntakeRequest(SuperstructureState goal) {
		if (!(goal.mType == SuperstructureState.Type.INTAKING)) {
			System.out.println("Wrong Goal Type");
			return new ParallelRequest();
		}
		return new SequentialRequest(
			mIntake.stateRequest(Intake.State.HALF_INTAKING),//idle indexer
			new ParallelRequest(
				mPivot.stateRequest(goal.mPivotState),
				mShooter.stateRequest(goal.mShooterState)),//FIXXUHHHHHH
			mIntake.stateRequest(goal.mIntakeState)
		).addName("Intaking");
	}

	/**
	 * Creates a request for scoring based on the goal state.
	 * 
	 * @param goal The goal state.
	 * @return A request for scoring.
	 */
	private Request ScoreGoalRequest(SuperstructureState goal) {
		if (!(goal.mType == SuperstructureState.Type.SCORING)) {
			System.out.println("Wrong Goal Type");
		}
		return new ParallelRequest(
			new ParallelRequest(
				mPivot.stateRequest(goal.mPivotState),
				mIntake.stateRequest(goal.mIntakeState),
				mShooter.stateRequest(Shooter.State.IDLE),
				new SequentialRequest(
					mPivot.waitToBeOverRequest(PivotConstants.kClearHeight),
			new SequentialRequest(
			ReadyToScoreRequest(),
			mShooter.stateRequest(goal.mShooterState))
		).addName("Score")));
	}

	/**
	 * Creates a request for scoring based on the goal state.
	 * 
	 * @param goal The goal state.
	 * @return A request for scoring.
	 */

	/**
	 * Creates a request to wait until the system is ready to score.
	 * 
	 * @return A request to wait until the system is ready to score.
	 */
	private Request ReadyToScoreRequest() {
		return (new Request() {
			@Override
			public void act() {}

			@Override
			public boolean isFinished() {
				return readyToScore;
			}
		}).addName("Driver Wait");
	}

	public void setReadyToScore(boolean newReady){
		readyToScore = newReady;
	}

	public void toggleAllowPoseComp(){
		driverAllowsPoseComp = !driverAllowsPoseComp;
	}

	public Request BooleanWaitRequest(BooleanSupplier booleanSupplier,boolean target){
		return new Request() {
			@Override
			public void act() {}

			@Override
			public boolean isFinished() {
				return target ? booleanSupplier.getAsBoolean() : !booleanSupplier.getAsBoolean();
			}
		};
	}
}