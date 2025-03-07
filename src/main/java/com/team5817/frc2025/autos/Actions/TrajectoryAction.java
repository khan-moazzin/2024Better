package com.team5817.frc2025.autos.Actions;

import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.lib.motion.Trajectory;

/**
 * TrajectoryAction is an action that sets a trajectory for the robot to follow.
 */
public class TrajectoryAction implements Action{

	private Drive mDrive = null;
	private Trajectory mTrajectory;

	/**
	 * Constructs a TrajectoryAction with the specified trajectory.
	 *
	 * @param path The trajectory to follow.
	 */
	public TrajectoryAction(Trajectory path){
		this(path, false);
	}

	/**
	 * Constructs a TrajectoryAction with the specified trajectory and reset position flag.
	 *
	 * @param path The trajectory to follow.
	 * @param resetPos Whether to reset the robot's position.
	 */
	public TrajectoryAction(Trajectory path, boolean resetPos){
		this.mTrajectory = path;
		this.mDrive = Drive.getInstance();
	}

	/**
	 * Starts the action by setting the trajectory in the drive subsystem.
	 */
	@Override
	public void start(){
		mDrive.setTrajectory(mTrajectory);
	}

	/**
	 * Checks if the trajectory is finished.
	 *
	 * @return true if the trajectory is finished, false otherwise.
	 */
	@Override
	public boolean isFinished() {
		// return false;
		return mDrive.isTrajectoryFinished();
	}

	/**
	 * Updates the action. This method is called periodically while the action is running.
	 */
	@Override
	public void update() {
	}

	/**
	 * Called once when the action is finished.
	 */
	@Override
	public void done() {
		System.out.println("Segement Complete");
		mDrive.setControlState(Drive.DriveControlState.OPEN_LOOP);
	}
}