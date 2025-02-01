package com.team5817.frc2025.autos.Modes;

import java.util.List;

import com.team5817.frc2025.Constants;
import com.team5817.frc2025.Robot;
import com.team5817.frc2025.autos.AutoBase;
import com.team5817.frc2025.autos.Actions.LambdaAction;
import com.team5817.frc2025.autos.Actions.ParallelAction;
import com.team5817.frc2025.autos.Actions.SequentialAction;
import com.team5817.frc2025.autos.Actions.TrajectoryAction;
import com.team5817.frc2025.autos.Actions.WaitAction;
import com.team5817.frc2025.autos.Actions.WaitToPassDistanceToReef;
import com.team5817.frc2025.autos.Actions.WaitForSuperstructureAction;
import com.team5817.frc2025.autos.TrajectoryLibrary.l;
import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Superstructure.GoalState;
import com.team5817.lib.motion.TrajectorySet;

public class ThreeCoralMode extends AutoBase {

	private Superstructure s = Superstructure.getInstance();
	private Drive d = Drive.getInstance();
	private TrajectorySet t;

	private double enterDistance = 3;
	private double exitDistance = 1.5;
	private double coralSpit = .1;

	public ThreeCoralMode(boolean mirror) {

		t = new TrajectorySet(
				mirror,
				l.TSTo3A,
				l.T3AToH,
				l.THTO7A,
				l.T7AToH,
				l.THTO7B);
		// if()j

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
						new LambdaAction(() -> {
							s.setGoal(GoalState.L4);
						}))))));

		System.out.println("Aligning to 7A");
		d.autoAlignFinishedOverrride(true);

		r(new WaitForSuperstructureAction());
		r(new WaitAction(coralSpit));

		System.out.println("Scored 7A");

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
						new LambdaAction(() -> {
							s.setGoal(GoalState.L4);
						}))))));

		System.out.println("Aligning to 7B");
		d.autoAlignFinishedOverrride(true);
		r(new WaitForSuperstructureAction());
		r(new WaitAction(coralSpit));
		System.out.println("Scored 7B");

	}
}