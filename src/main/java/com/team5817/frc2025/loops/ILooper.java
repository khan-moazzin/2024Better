package com.team5817.frc2025.loops;

/**
 * Interface for registering loops.
 */
public interface ILooper {
    /**
     * Registers a loop to be run.
     *
     * @param loop the loop to register
     */
    void register(Loop loop);
}
