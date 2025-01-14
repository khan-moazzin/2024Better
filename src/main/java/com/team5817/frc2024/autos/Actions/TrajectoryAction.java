package com.team5817.frc2024.autos.Actions;


import com.team254.lib.trajectory.TrajectoryIterator;
import com.team5817.frc2024.subsystems.Drive.Drive;
import com.team5817.lib.motion.Trajectory;



public class TrajectoryAction implements Action{

	private Drive mDrive = null;


	private TrajectoryIterator mTrajectory;

	public TrajectoryAction(Trajectory path){
		this(path, false);
	}

	public TrajectoryAction(Trajectory path, boolean resetPos){
		this.mTrajectory = path.get();
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