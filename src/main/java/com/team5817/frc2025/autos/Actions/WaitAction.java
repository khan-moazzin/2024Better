package com.team5817.frc2025.autos.Actions;

import edu.wpi.first.wpilibj.Timer;

/**
 * Action to wait for a given amount of time.
 * To use this Action, call runAction(new WaitAction(your_time)).
 */
public class WaitAction implements Action {

	private double mTimeToWait;
	private double mStartTime;

	/**
	 * Constructor for WaitAction.
	 * 
	 * @param timeToWait The amount of time to wait in seconds.
	 */
	public WaitAction(double timeToWait) {
		mTimeToWait = timeToWait;
	}

	/**
	 * Checks if the wait time has elapsed.
	 * 
	 * @return true if the wait time has elapsed, false otherwise.
	 */
	@Override
	public boolean isFinished() {
		return Timer.getFPGATimestamp() - mStartTime >= mTimeToWait;
	}

	/**
	 * Updates the action. This method is called periodically while the action is running.
	 */
	@Override
	public void update() {}

	/**
	 * Called once when the action is finished.
	 */
	@Override
	public void done() {}

	/**
	 * Starts the action and records the start time.
	 */
	@Override
	public void start() {
		mStartTime = Timer.getFPGATimestamp();
	}
}
