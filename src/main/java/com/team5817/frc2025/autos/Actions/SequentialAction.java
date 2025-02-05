package com.team5817.frc2025.autos.Actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * An action that runs a sequence of actions one after another.
 */
public class SequentialAction implements Action {
	private Action mCurrentAction;
	private final ArrayList<Action> mRemainingActions;

	/**
	 * Constructs a SequentialAction with a list of actions.
	 *
	 * @param actions the list of actions to run sequentially
	 */
	public SequentialAction(List<Action> actions) {
		mRemainingActions = new ArrayList<>(actions.size());
		mRemainingActions.addAll(actions);
		mCurrentAction = null;
	}

	/**
	 * Constructs a SequentialAction with an array of actions.
	 *
	 * @param actions the array of actions to run sequentially
	 */
	public SequentialAction(Action... actions) {
		this(Arrays.asList(actions));
	}

	/**
	 * Starts the sequential action.
	 */
	@Override
	public void start() {}

	/**
	 * Updates the current action. If the current action is finished, it moves to the next action.
	 */
	@Override
	public void update() {
		if (mCurrentAction == null) {
			if (mRemainingActions.isEmpty()) {
				return;
			}

			mCurrentAction = mRemainingActions.remove(0);
			mCurrentAction.start();
		}

		mCurrentAction.update();

		if (mCurrentAction.isFinished()) {
			mCurrentAction.done();
			mCurrentAction = null;
		}
	}

	/**
	 * Checks if all actions are finished.
	 *
	 * @return true if all actions are finished, false otherwise
	 */
	@Override
	public boolean isFinished() {
		return mRemainingActions.isEmpty() && mCurrentAction == null;
	}

	/**
	 * Called once when the action is finished.
	 */
	@Override
	public void done() {}
}
