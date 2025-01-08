package com.team5817.frc2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class VisionFrame {
	double timestamp;
	double[] frame_data;

	// For comparator
	public double getTimestamp() {
		return timestamp;
	}
}
