package com.team5817.frc2025.autos.Actions;

import java.util.function.BooleanSupplier;


/**
 * Action that waits for a specific input from the controller.
 */
public class WaitForBooleanAction implements Action{

    public WaitForBooleanAction(BooleanSupplier supplier, boolean target){
        this.supplier = supplier;
    }
    public WaitForBooleanAction(BooleanSupplier supplier){
        this(supplier,true);
    }
    BooleanSupplier supplier;
    boolean target;
    /**
     * Initializes the controller.
     */
    @Override
    public void start() {
    }

    /**
     * Updates the controller state.
     */
    @Override
    public void update() {
    }

    /**
     * Cleans up the controller.
     */
    @Override
    public void done() {
    }

    /**
     * Checks if the A button has been pressed.
     * 
     * @return true if the A button is pressed, false otherwise.
     */
    @Override
    public boolean isFinished() {
        if(target)
            return supplier.getAsBoolean();
        else
            return !supplier.getAsBoolean();
    }

}
