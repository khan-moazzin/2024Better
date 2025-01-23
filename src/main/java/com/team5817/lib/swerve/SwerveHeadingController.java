package com.team5817.lib.swerve;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.SynchronousPIDF;
import com.team5817.frc2025.Constants.SwerveConstants;

import edu.wpi.first.wpilibj.Timer;

public class SwerveHeadingController {
	private static SwerveHeadingController mInstance;

	public static SwerveHeadingController getInstance() {
		if (mInstance == null) {
			mInstance = new SwerveHeadingController();
		}

		return mInstance;
	}

	public Rotation2d targetHeadingRadians;
	private double lastUpdatedTimestamp;

	public enum State {
		OFF,
		SNAP,
		STABILIZE
	}

	private State current_state = State.OFF;

	public State getState() {
		return current_state;
	}

	public void setState(State state) {
		current_state = state;
	}

	public void disable() {
		setState(State.OFF);
	}

	private SynchronousPIDF stabilizePID;
	private SynchronousPIDF snapPID;

	public void setSnapTarget(Rotation2d angle_rad) {
		targetHeadingRadians = angle_rad;
		setState(State.SNAP);
	}

	public void setStabilizeTarget(Rotation2d angle_rad) {
		targetHeadingRadians = angle_rad;
		setState(State.STABILIZE);
	}

	public Rotation2d getTargetHeading() {
		return targetHeadingRadians;
	}

	public SwerveHeadingController() {
		stabilizePID = new SynchronousPIDF(
				SwerveConstants.kStabilizeSwerveHeadingKp,
				SwerveConstants.kStabilizeSwerveHeadingKi,
				SwerveConstants.kStabilizeSwerveHeadingKd,
				SwerveConstants.kStabilizeSwerveHeadingKf);

		snapPID = new SynchronousPIDF(
				SwerveConstants.kSnapSwerveHeadingKp,
				SwerveConstants.kSnapSwerveHeadingKi,
				SwerveConstants.kSnapSwerveHeadingKd,
				SwerveConstants.kSnapSwerveHeadingKf);

		stabilizePID.setInputRange(-Math.PI, Math.PI);
		stabilizePID.setContinuous();

		stabilizePID.setOutputRange(-10 * Math.PI, 10 * Math.PI);

		targetHeadingRadians = Rotation2d.identity();
		lastUpdatedTimestamp = Timer.getFPGATimestamp();
	}

	public double update(Rotation2d heading, double timestamp) {
		double correction = 0;
		double error = heading.minus(targetHeadingRadians).getRadians();
		double dt = timestamp - lastUpdatedTimestamp;
		switch (current_state) {
			case OFF:
				break;
			case STABILIZE:
				correction = stabilizePID.calculate(error, dt);
				break;
			case SNAP:
				correction = snapPID.calculate(error, dt);
				break;
		}

		lastUpdatedTimestamp = timestamp;
		return correction;
	}
}
