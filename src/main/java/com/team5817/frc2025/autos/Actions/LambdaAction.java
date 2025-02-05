package com.team5817.frc2025.autos.Actions;

/**
 * An action that executes a lambda function.
 */
public class LambdaAction implements Action {

    /**
     * Functional interface for a void function.
     */
    public interface VoidInterace {
        void function();
    }

    private final VoidInterace mFunction;

    /**
     * Constructs a LambdaAction with the specified function.
     * 
     * @param function the function to execute
     */
    public LambdaAction(VoidInterace function) {
        this.mFunction = function;
    }

    /**
     * Called when the action starts and executes the function.
     */
    @Override
    public void start() {
        mFunction.function();
    }

    /**
     * Called periodically to update the action.
     */
    @Override
    public void update() {}

    /**
     * Checks if the action is finished.
     * 
     * @return true since this action does nothing after the function is executed
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
