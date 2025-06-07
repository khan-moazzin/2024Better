package com.team5817.frc2025.autos.Actions;

import com.team5817.frc2025.controls.CustomXboxController;

/**
 * Action that waits for a specific input from the controller.
 */
public class WaitforControllerInput implements Action{

    CustomXboxController mController;

    /**
     * Initializes the controller.
     */
    @Override
    public void start() {
        mController = new CustomXboxController(0);
    }

    /**
     * Updates the controller state.
     */
    @Override
    public void update() {
        mController.update();
    }

    /**
     * Cleans up the controller.
     */
    @Override
    public void done() {
        mController = null;
    }

    /**
     * Checks if the A button has been pressed.
     * 
     * @return true if the A button is pressed, false otherwise.
     */
    @Override
    public boolean isFinished() {
        return mController.getAButtonPressed();
    }

}
