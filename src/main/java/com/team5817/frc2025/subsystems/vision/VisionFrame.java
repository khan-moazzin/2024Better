package com.team5817.frc2025.subsystems.vision;

/**
 * Represents a frame of vision data with a timestamp.
 */
public class VisionFrame {
	double timestamp;
	double[] frame_data;

	/**
	 * Gets the timestamp of the vision frame.
	 *
	 * @return the timestamp of the vision frame
	 */
	public double getTimestamp() {
		return timestamp;
	}
}
