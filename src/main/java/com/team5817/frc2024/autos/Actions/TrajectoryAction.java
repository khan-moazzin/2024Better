package com.team5817.frc2024.autos.Actions;


import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team5817.frc2024.subsystems.Drive.Drive;
import com.team5817.lib.motion.PPTimeView;



public class TrajectoryAction implements Action{

	private Drive mDrive = null;


	private TrajectoryIterator mTrajectory;

	public TrajectoryAction(PathPlannerTrajectory path){
		this(path, false);
	}

	public TrajectoryAction(PathPlannerTrajectory path, boolean resetPos){
		this.mTrajectory = new TrajectoryIterator(new PPTimeView(path));
		this.mDrive = Drive.getInstance();
	}

	@Override
	public void start(){
		// mDrive.setTrajectory(mTrajectory);
	}

	@Override
	public boolean isFinished() {
		return false;
		// return mDrive.isTrajectoryFinished();
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {
		// mDrive.setState(State.OFF);
	}
}