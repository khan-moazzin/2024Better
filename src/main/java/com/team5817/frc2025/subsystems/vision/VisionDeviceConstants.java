package com.team5817.frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * Constants for configuring a vision device.
 */
public class VisionDeviceConstants {
	/**
	 * The name of the network table for the vision device.
	 */
	public String kTableName = "";
	
	/**
	 * The transform from the robot to the camera.
	 */
	public Transform3d kRobotToCamera = new Transform3d();
	
	/**
	 * The ID of the camera.
	 */
	public int kCameraId = 0;
	
	/**
	 * The width of the camera resolution.
	 */
	public int kCameraResolutionWidth = 1600;
	
	/**
	 * The height of the camera resolution.
	 */
	public int kCameraResolutionHeight = 1200;
}
