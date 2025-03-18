package com.team5817.frc2025.autos.Modes;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.team5817.frc2025.Constants;
import com.team5817.frc2025.autos.AutoBase;
import com.team5817.frc2025.autos.Actions.LambdaAction;
import com.team5817.frc2025.autos.Actions.ParallelAction;
import com.team5817.frc2025.autos.Actions.SequentialAction;
import com.team5817.frc2025.autos.Actions.TrajectoryAction;
import com.team5817.frc2025.autos.Actions.WaitAction;
import com.team5817.frc2025.autos.Actions.WaitToPassDistanceToReef;
import com.team5817.frc2025.autos.AutoModeSelector.PickupLocation;
import com.team5817.frc2025.autos.AutoModeSelector.ScoringLocation;
import com.team5817.frc2025.autos.AutoModeSelector.StartingPosition;
import com.team5817.frc2025.autos.Actions.WaitForSuperstructureAction;
import com.team5817.frc2025.autos.TrajectoryLibrary.l;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Superstructure.GoalState;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;

import edu.wpi.first.wpilibj.Timer;

/**
 * CustomThreeCoralMode is an autonomous mode for handling three coral scoring actions.
 */
public class CustomGroundMode extends AutoBase {

	private Superstructure s = Superstructure.getInstance();
	private Drive d = Drive.getInstance();
	private TrajectorySet t;
	private int coral_amount = 0;

	private double enterDistance = 3;
	private double exitDistance = 1.5;
	private double coralSpit = .25;
	private double intakeWait = 0;
	private double alignWait = 1;
	private double intakeTimeout = 0;
	private double scoreTimeout = 1;

	private String firstScoreName;
	private String secondScoreName;
	private String thirdScoreName;

	private PickupLocation firstPickup;
	private PickupLocation secondPickup;
	private PickupLocation thirdPickup;

	/**
	 * Constructs a CustomThreeCoralMode with the specified starting position, pickup location, and scoring locations.
	 *
	 * @param startingPosition the starting position of the robot
	 * @param thirdPickup the location to pick up corals
	 * @param firstScore the first scoring location
	 * @param secondScore the second scoring location
	 * @param thirdScore the third scoring location
	 */
	public CustomGroundMode(StartingPosition startingPosition, PickupLocation firstPickup, PickupLocation secondPickup, PickupLocation thirdPickup, ScoringLocation firstScore, ScoringLocation secondScore, ScoringLocation thirdScore, int scoreAmount) {

 	boolean mirror = startingPosition.mirrored;
	Trajectory startingTrajectory;
	Trajectory firstPickupTrajectory;
	Trajectory secondScoreTrajectory;
	Trajectory secondPickupTrajectory;
	Trajectory thirdScoreTrajectory;
	Trajectory thirdPickupTrajectory;


	firstScoreName = firstScore.toString();
	secondScoreName = secondScore.toString();
	thirdScoreName = thirdScore.toString();
	
	startingTrajectory = l.trajectories.get(startingPosition.name + "To" + firstScoreName);
	firstPickupTrajectory = l.trajectories.get(firstScoreName + "To" + firstPickup.name);

	secondScoreTrajectory = l.trajectories.get(firstPickup.name + "To" + secondScoreName);
	secondPickupTrajectory = l.trajectories.get(secondScoreName + "To" + secondPickup.name);

	thirdScoreTrajectory = l.trajectories.get(secondPickup.name + "To" + thirdScoreName);
	thirdPickupTrajectory = l.trajectories.get(thirdScoreName + "To" + thirdPickup.name);

	this.firstPickup = firstPickup;
	this.secondPickup = secondPickup;
	this.thirdPickup = thirdPickup;

	 
	t = new TrajectorySet(
			mirror,
			startingTrajectory,
			firstPickupTrajectory,
			secondScoreTrajectory,
			secondPickupTrajectory,
			thirdScoreTrajectory,
			thirdPickupTrajectory
			);
	}

	/**
	 * Executes the autonomous routine for scoring three corals.
	 */
	@Override
	public void routine() {
		if (Constants.mode == Constants.Mode.SIM) {
			mSim.setSimulationWorldPose(t.initalPose().wpi());
		}
		s.setReadyToScore(false);
		r(new ParallelAction(List.of(
				new TrajectoryAction(t.next(),scoreTimeout),
				new SequentialAction(List.of(
						new LambdaAction(() -> {
							s.setGoal(GoalState.L4);
						}))))));
		r(new WaitAction(alignWait));
		System.out.println("Starting Score " + firstScoreName+" at "+ (Timer.getTimestamp()-startTime));
		s.setReadyToScore(true);

		r(new WaitForSuperstructureAction());
		r(new WaitAction(coralSpit));
		if(coral_amount==1)
				return;

		System.out.println("Scored " + firstScoreName+" at "+ (Timer.getTimestamp()-startTime));
		r(new ParallelAction(List.of(
				new TrajectoryAction(t.next(),intakeTimeout),
				new SequentialAction(List.of(
						new WaitToPassDistanceToReef(exitDistance),
						new LambdaAction(() -> {
							s.setGoal(firstPickup.state);
						}))))));
		
		r(new WaitAction(intakeWait));
		System.out.println("Collected 1st coral"+" at "+ (Timer.getTimestamp()-startTime));
		s.setGoal(GoalState.STOW);
		s.setReadyToScore(false);
		r(new ParallelAction(List.of(
				new TrajectoryAction(t.next(),scoreTimeout),
				new SequentialAction(List.of(
						new WaitToPassDistanceToReef(enterDistance),
						new LambdaAction(() -> {
							s.setGoal(GoalState.L4);
						}))))));
		r(new WaitAction(alignWait));

		System.out.println("Starting Score " + secondScoreName+" at "+ (Timer.getTimestamp()-startTime));
		s.setReadyToScore(true);

		r(new WaitForSuperstructureAction());
		r(new WaitAction(coralSpit));
		System.out.println("Scored " + secondScoreName+" at "+ (Timer.getTimestamp()-startTime));
		if(coral_amount ==2)
			return;

		r(new ParallelAction(List.of(
				new TrajectoryAction(t.next(),intakeTimeout),
				new SequentialAction(List.of(
						new WaitToPassDistanceToReef(exitDistance),
						new LambdaAction(() -> {
							s.setGoal(secondPickup.state);
						}))))));

		r(new WaitAction(intakeWait));
		System.out.println("Collected 2nd coral"+" at "+ (Timer.getTimestamp()-startTime));
		s.setGoal(GoalState.STOW);

		s.setReadyToScore(false);
		r(new ParallelAction(List.of(
				new TrajectoryAction(t.next(),scoreTimeout),
				new SequentialAction(List.of(
						new WaitToPassDistanceToReef(enterDistance),
						new LambdaAction(() -> {
							s.setGoal(GoalState.L4);
						}))))));
		r(new WaitAction(alignWait));

		System.out.println("Starting Score " + thirdScoreName+" at "+ (Timer.getTimestamp()-startTime));
		s.setReadyToScore(true);
		r(new WaitForSuperstructureAction());
		r(new WaitAction(coralSpit));
		System.out.println("Scored " + thirdScoreName+" at "+ (Timer.getTimestamp()-startTime));

		r(new ParallelAction(List.of(
				new TrajectoryAction(t.next(),intakeTimeout),
				new SequentialAction(List.of(
						new WaitToPassDistanceToReef(exitDistance),
						new LambdaAction(() -> {
							s.setGoal(thirdPickup.state);
						}))))));
					}
}