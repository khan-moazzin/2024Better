package com.team5817.frc2025.loops;

/**
 * Interface for loops, which are routine that run periodically in the robot code (such as periodic gyroscope
 * calibration, etc.)
 */
public interface Loop {

    /**
     * Method to be called when the loop starts.
     *
     * @param timestamp The current time in seconds.
     */
    public void onStart(double timestamp);

    /**
     * Method to be called periodically during the loop.
     *
     * @param timestamp The current time in seconds.
     */
    public void onLoop(double timestamp);
}
