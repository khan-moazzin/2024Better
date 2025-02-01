package com.team5817.frc2025.autos.Actions;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


public class SequentialAction implements Action {
	private Action mCurrentAction;
	private final ArrayList<Action> mRemainingActions;

	public SequentialAction(List<Action> actions) {
		mRemainingActions = new ArrayList<>(actions.size());
		mRemainingActions.addAll(actions);
		mCurrentAction = null;
	}

	public SequentialAction(Action... actions) {
		this(Arrays.asList(actions));
	}

	@Override
	public void start() {}

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

	@Override
	public boolean isFinished() {
		return mRemainingActions.isEmpty() && mCurrentAction == null;
	}

	@Override
	public void done() {}
}
