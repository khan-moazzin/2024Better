package com.team5817.lib.swerve;

import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.SynchronousPIDF;
import com.team5817.frc2025.subsystems.Drive.SwerveConstants;
import com.team5817.lib.RobotMode;

import edu.wpi.first.wpilibj.Timer;

public class SwerveHeadingController {
	private static SwerveHeadingController mInstance;
	private Rotation2d targetHeading = Rotation2d.fromDegrees(0);


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
    private double lastTimestamp;

	private double disabledStartTimestamp = 0;
    private boolean atTarget = false;
    private boolean isDisabled = false;
	private SynchronousPIDF openLoopController;
    public void disableSwerveHeadingController(boolean disable) {
        disabledStartTimestamp = lastTimestamp;
        isDisabled = disable;
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

    public void setTargetHeading(Rotation2d heading) {
        targetHeading = Rotation2d.fromDegrees(heading.getDegrees());
        atTarget = false;
    }

	public double updateRotationCorrection(Rotation2d heading, double timestamp) {
        if(isDisabled) {
            if((timestamp - disabledStartTimestamp) > 0.25) {
                isDisabled = false;
                setTargetHeading(heading);
            }
            return 0;
        }
        return getRotationCorrection(heading, timestamp);
    }

	public double getRotationCorrection(Rotation2d heading, double timestamp) {
        double error = new Rotation2d(targetHeading).rotateBy(heading.inverse()).getDegrees();
        double dt = timestamp - lastTimestamp;
        lastTimestamp = timestamp;
        if(Math.abs(error)< 2.5){
            atTarget = true;
        }
        double correctionForce = openLoopController.calculate(error, dt);
        if(Math.abs(correctionForce) > 0.017){
            correctionForce = .0075* Math.signum(correctionForce);
        }
        return correctionForce;
    }

    public boolean atTarget(){
        return atTarget;
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

				if (RobotMode.mode == RobotMode.Mode.SIM) {
					stabilizePID.setPIDF(
						SwerveConstants.kStabilizeSwerveHeadingKp * 2,
						SwerveConstants.kStabilizeSwerveHeadingKi * 2,
						SwerveConstants.kStabilizeSwerveHeadingKd * 2,
						SwerveConstants.kStabilizeSwerveHeadingKf * 2
					);

					snapPID.setPIDF(
						SwerveConstants.kSnapSwerveHeadingKp * 2,
						SwerveConstants.kSnapSwerveHeadingKi * 2,
						SwerveConstants.kSnapSwerveHeadingKd * 2,
						SwerveConstants.kSnapSwerveHeadingKf * 2
					);
				}

		stabilizePID.setInputRange(-Math.PI, Math.PI);
		stabilizePID.setContinuous();

		stabilizePID.setOutputRange(-10 * Math.PI, 10 * Math.PI);

		snapPID.setInputRange(-Math.PI, Math.PI);
		snapPID.setContinuous();

		snapPID.setOutputRange(-10 * Math.PI, 10 * Math.PI);
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
