package com.team5817.frc2025.autos.Modes;

import java.util.List;

import com.team5817.frc2025.Constants;
import com.team5817.frc2025.autos.AutoBase;
import com.team5817.frc2025.autos.Actions.LambdaAction;
import com.team5817.frc2025.autos.Actions.ParallelAction;
import com.team5817.frc2025.autos.Actions.SequentialAction;
import com.team5817.frc2025.autos.Actions.TrajectoryAction;
import com.team5817.frc2025.autos.Actions.WaitAction;
import com.team5817.frc2025.autos.Actions.WaitToPassDistanceToReef;
import com.team5817.frc2025.autos.AutoModeSelector.PickupLocation;
import com.team5817.frc2025.autos.AutoModeSelector.StartingPosition;
import com.team5817.frc2025.autos.Actions.WaitForSuperstructureAction;
import com.team5817.frc2025.autos.TrajectoryLibrary.l;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Superstructure.GoalState;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;

public class ThreeCoralMode extends AutoBase {

	private Superstructure s = Superstructure.getInstance();
	private Drive d = Drive.getInstance();
	private TrajectorySet t;

	private double enterDistance = 3;
	private double exitDistance = 1.5;
	private double coralSpit = .1;

	public ThreeCoralMode(StartingPosition startingPosition, PickupLocation pickupLocation) {

 	boolean mirror = startingPosition.mirrored;
	Trajectory startingTrajectory;
	Trajectory followUpTrajectory;

	switch (startingPosition) {
		case CENTER_PROCESS:
		case CENTER_BLANK:
			startingTrajectory = l.TCTo3A;
			followUpTrajectory = l.T3AToFH;
			break;
		case BLANK_SIDE:
		case PROCCESSOR_SIDE:

			startingTrajectory = l.TSTo8A;
			
			followUpTrajectory = l.T8AToFH;
		break;
		default:
			startingTrajectory = l.TCTo3A;
			followUpTrajectory = l.T3AToFH;
		break;

	}
		t = new TrajectorySet(
				mirror,
				startingTrajectory,
				followUpTrajectory,
				l.TFHTo7A,
				l.T7AToFH,
				l.TFHTo7B,
				l.T7BToFH
				);
	}

	// spotless:off
	@Override
	public void routine() {
		if (Constants.mode == Constants.Mode.SIM) {
			mSim.setSimulationWorldPose(t.initalPose().wpi());
		}
	
		d.autoAlignFinishedOverrride(false);
		r(new ParallelAction(List.of(
				new TrajectoryAction(t.next()),
				new SequentialAction(List.of(
						new LambdaAction(() -> {
							s.setGoal(GoalState.L4);
						}))))));
		System.out.println("Aligning to 3A");
		d.autoAlignFinishedOverrride(true);

		r(new WaitForSuperstructureAction());
		r(new WaitAction(coralSpit));

		System.out.println("Scored 3A");
		r(new ParallelAction(List.of(
				new TrajectoryAction(t.next()),
				new SequentialAction(List.of(
						new WaitToPassDistanceToReef(exitDistance),
						new LambdaAction(() -> {
							s.setGoal(GoalState.HUMAN_CORAL_INTAKE);
						}))))));
		System.out.println("Collected 1st coral");

		d.autoAlignFinishedOverrride(false);
		r(new ParallelAction(List.of(
				new TrajectoryAction(t.next()),
				new SequentialAction(List.of(
						new WaitToPassDistanceToReef(enterDistance),
						// new LambdaAction(()-> d.setUseSpecializedPoseForPath(true)),
						new LambdaAction(() -> {
							s.setGoal(GoalState.L4);
						}))))));

		System.out.println("Aligning to 7A");
		d.autoAlignFinishedOverrride(true);

		r(new WaitForSuperstructureAction());
		r(new WaitAction(coralSpit));
		System.out.println("Scored 7A");
		// d.setUseSpecializedPoseForPath(false);

		r(new ParallelAction(List.of(
				new TrajectoryAction(t.next()),
				new SequentialAction(List.of(
						new WaitToPassDistanceToReef(exitDistance),
						new LambdaAction(() -> {
							s.setGoal(GoalState.HUMAN_CORAL_INTAKE);
						}))))));
		System.out.println("Collected 2nd coral");

		d.autoAlignFinishedOverrride(false);
		r(new ParallelAction(List.of(
				new TrajectoryAction(t.next()),
				new SequentialAction(List.of(
						new WaitToPassDistanceToReef(enterDistance),
						// new LambdaAction(() -> d.setUseSpecializedPoseForPath(true)),
						new LambdaAction(() -> {
							s.setGoal(GoalState.L4);
						}))))));

		System.out.println("Aligning to 7B");
		d.autoAlignFinishedOverrride(true);
		r(new WaitAction(coralSpit));
		System.out.println("Scored 7B");

		// d.setUseSpecializedPoseForPath(false);

		r(new ParallelAction(List.of(
				new TrajectoryAction(t.next()),
				new SequentialAction(List.of(
						new WaitToPassDistanceToReef(exitDistance),
						new LambdaAction(() -> {
							s.setGoal(GoalState.HUMAN_CORAL_INTAKE);
						}))))));
	
		

	}
}