package com.team5817.frc2025.autos.Actions;

/**
 * An empty action that does nothing.
 */
public class EmptyAction implements Action {

    /**
     * Called when the action starts.
     */
    @Override
    public void start() {}

    /**
     * Called periodically to update the action.
     */
    @Override
    public void update() {}

    /**
     * Checks if the action is finished.
     * 
     * @return true since this action does nothing and is always finished.
     */
    @Override
    public boolean isFinished() {
        return true;
    }

    /**
     * Called when the action is done.
     */
    @Override
    public void done() {}
}
