package com.team5817.frc2025.autos.Modes;

import com.team5817.frc2025.autos.AutoBase;

/**
 * A mode that does nothing during the autonomous period.
 */
public class DoNothingMode extends AutoBase{

    /**
     * Constructs a new DoNothingMode.
     */
	public DoNothingMode() {
	}

	// spotless:off
	/**
     * The routine that runs during the autonomous period.
     * This routine does nothing.
     */
	@Override
	public void routine() {
        System.out.println("Did Nothing");
        System.out.println("Finished auto!");
	}
	// spotless:on
   
}
