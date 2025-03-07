package com.team5817.frc2025.autos.Actions;

import java.util.ArrayList;
import java.util.List;

/**
 * Composite action, running all sub-actions at the same time. 
 * All actions are started then updated until all actions report being done.
 */
public class ParallelAction implements Action {

    private final ArrayList<Action> mActions;

    /**
     * Constructs a ParallelAction with the specified list of actions.
     * 
     * @param actions the list of actions to run in parallel
     */
    public ParallelAction(List<Action> actions) {
        mActions = new ArrayList<>(actions);
    }

    /**
     * Checks if all sub-actions are finished.
     * 
     * @return true if all sub-actions are finished, false otherwise
     */
    @Override
    public boolean isFinished() {
        for (Action action : mActions) {
            if (!action.isFinished()) {
                return false;
            }
        }
        return true;
    }

    /**
     * Updates all sub-actions.
     */
    @Override
    public void update() {
        for (Action action : mActions) {
            action.update();
        }
    }

    /**
     * Calls the done method on all sub-actions.
     */
    @Override
    public void done() {
        for (Action action : mActions) {
            action.done();
        }
    }

    /**
     * Starts all sub-actions.
     */
    @Override
    public void start() {
        for (Action action : mActions) {
            action.start();
        }
    }
}
