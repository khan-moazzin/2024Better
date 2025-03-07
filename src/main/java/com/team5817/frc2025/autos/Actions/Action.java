package com.team5817.frc2025.autos.Actions;

/**
 * Interface representing an action to be performed.
 */
public interface Action {
	/**
	 * Checks if the action is finished.
	 * 
	 * @return True if the action is finished, false otherwise.
	 */
	boolean isFinished();

	/**
	 * Updates the action. This method is called periodically while the action is active.
	 */
	void update();

	/**
	 * Called once when the action is finished.
	 */
	void done();

	/**
	 * Called once when the action is started.
	 */
	void start();
}
