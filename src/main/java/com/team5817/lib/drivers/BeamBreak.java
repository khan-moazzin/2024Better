package com.team5817.lib.drivers;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * The BeamBreak class represents a beam break sensor.
 * It provides methods to check if the beam was tripped or cleared.
 */
public class BeamBreak {

	private boolean lastStatus;
	private boolean tripped;
	private boolean cleared;
	private final DigitalInput mBreak;

	/**
	 * Constructs a BeamBreak object.
	 *
	 * @param channel the digital input channel the beam break sensor is connected to
	 */
	public BeamBreak(int channel) {
		mBreak = new DigitalInput(channel);
	}

	/**
	 * Updates the status of the beam break sensor.
	 * This method should be called periodically to update the tripped and cleared states.
	 */
	public void update() {
		boolean value = get();
		tripped = value && !lastStatus;
		cleared = !value && lastStatus;
		lastStatus = value;
	}

	/**
	 * Gets the current status of the beam break sensor.
	 *
	 * @return true if the beam is broken, false otherwise
	 */
	public boolean get() {
		return !mBreak.get();
	}

	/**
	 * Checks if the beam was tripped since the last update.
	 *
	 * @return true if the beam was tripped, false otherwise
	 */
	public boolean wasTripped() {
		return tripped;
	}

	/**
	 * Checks if the beam was cleared since the last update.
	 *
	 * @return true if the beam was cleared, false otherwise
	 */
	public boolean wasCleared() {
		return cleared;
	}
}
