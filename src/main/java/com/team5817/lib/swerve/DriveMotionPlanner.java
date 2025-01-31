package com.team5817.lib.swerve;

import com.pathplanner.lib.path.PathPlannerPath;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team5817.frc2025.Constants.SwerveConstants;
import com.team5817.lib.motion.PPPathPointState;

/**
 * Class responsible for planning the motion of the drive system.
 */
public class DriveMotionPlanner {
	// Pure Pursuit Constants
	public static final double kPathLookaheadTime = 0.1; // From 1323 (2019)
	public static final double kPathMinLookaheadDistance = 0.3; // From 1323 (2019)
	public static final double kAdaptivePathMinLookaheadDistance = 0.15;
	public static final double kAdaptivePathMaxLookaheadDistance = 0.61;
	public static final double kAdaptiveErrorLookaheadCoefficient = 0.01;

	public enum FollowerType {
		PID,
		PURE_PURSUIT,
	}

	FollowerType mFollowerType = FollowerType.PID;

	/**
	 * Sets the follower type for the motion planner.
	 * 
	 * @param type The follower type to be set.
	 */
	public void setFollowerType(FollowerType type) {
		mFollowerType = type;
	}

	TrajectoryIterator mCurrentTrajectory;
	boolean mIsReversed = false;
	double mLastTime = Double.POSITIVE_INFINITY;
	PPPathPointState mLastSetpoint = null;
	PPPathPointState mSetpoint = new PPPathPointState();
	Pose2d mError = Pose2d.identity();

	Translation2d mTranslationalError = Translation2d.identity();
	Rotation2d mPrevHeadingError = Rotation2d.identity();
	Pose2d mCurrentState = Pose2d.identity();

	double mCurrentTrajectoryLength = 0.0;
	double mTotalTime = Double.POSITIVE_INFINITY;
	double mStartTime = Double.POSITIVE_INFINITY;
	ChassisSpeeds mOutput = new ChassisSpeeds();
	boolean mPathIsFinished = false;

	// PID controllers for path following
	double mDt = 0.0;

	public DriveMotionPlanner() {
	}

	/**
	 * Sets the trajectory for the motion planner.
	 * 
	 * @param trajectory The trajectory to be followed.
	 */
	public void setTrajectory(final TrajectoryIterator trajectory) {
		mCurrentTrajectory = trajectory;
		mSetpoint = trajectory.getCurrentState();
		mLastSetpoint = null;
		mCurrentTrajectoryLength = mCurrentTrajectory.getTimeView().last_interpolant();
	}

	/**
	 * Resets the motion planner to its initial state.
	 */
	public void reset() {
		mTranslationalError = Translation2d.identity();
		mPrevHeadingError = Rotation2d.identity();
		mLastSetpoint = null;
		mLastTime = Double.POSITIVE_INFINITY;
		mPathIsFinished = false;
	}

	/**
	 * Updates the chassis speeds using PID control.
	 * 
	 * @param chassisSpeeds The current chassis speeds.
	 * @return The updated chassis speeds.
	 */
	public ChassisSpeeds updatePIDChassis(ChassisSpeeds chassisSpeeds) {
		final double kPathk = 2.4;
		Twist2d pid_error = Pose2d.log(mError);
		chassisSpeeds.vxMetersPerSecond = (chassisSpeeds.vxMetersPerSecond) + kPathk * pid_error.dx;
		chassisSpeeds.vyMetersPerSecond = (chassisSpeeds.vyMetersPerSecond) + kPathk * pid_error.dy;
		chassisSpeeds.omegaRadiansPerSecond = chassisSpeeds.omegaRadiansPerSecond + kPathk * pid_error.dtheta;
		return chassisSpeeds;
	}

	/**
	 * Updates the motion planner with the current timestamp and state.
	 * 
	 * @param timestamp     The current timestamp.
	 * @param current_state The current state of the robot.
	 * @return The updated chassis speeds.
	 */
	public ChassisSpeeds update(double timestamp, Pose2d current_state) {
		if (mCurrentTrajectory == null)
			return null;
		if (!Double.isFinite(mLastTime))
			mLastTime = timestamp;
		mDt = timestamp - mLastTime;
		mLastTime = timestamp;
		PPPathPointState sample_point;
		mCurrentState = current_state;

		// Compute error in robot frame
		mPrevHeadingError = mError.getRotation();
		mError = current_state.getPose().inverse().transformBy(mSetpoint.getPose());
		sample_point = mCurrentTrajectory.advance(mDt);
		mSetpoint = sample_point;

		var chassis_speeds = new ChassisSpeeds(
				sample_point.getXVel(),
				sample_point.getYVel(),
				sample_point.getThetaVel());
		mOutput = updatePIDChassis(chassis_speeds);
		mPathIsFinished = distance(current_state, mCurrentTrajectoryLength) < SwerveConstants.kTrajectoryDeadband;
		mOutput = ChassisSpeeds.fromFieldRelativeSpeeds(chassis_speeds, current_state.getRotation());
		return mOutput;
	}

	/**
	 * Calculates the distance to the end of the trajectory.
	 * 
	 * @param current_state       The current state of the robot.
	 * @param additional_progress Additional progress along the trajectory.
	 * @return The distance to the end of the trajectory.
	 */
	private double distance(Pose2d current_state, double additional_progress) {
		return mCurrentTrajectory
				.preview(additional_progress)
				.getPose()
				.distance(current_state);
	}

	/**
	 * Gets the current path being followed.
	 * 
	 * @return The current path.
	 */
	public PathPlannerPath getPath() {
		if (mCurrentTrajectory == null)
			return null;
		return mCurrentTrajectory.getPath();
	}

	/**
	 * Gets the end position of the current trajectory.
	 * 
	 * @return The end position.
	 */
	public Pose2d getEndPosition() {
		return mCurrentTrajectory.getTimeView().sample(mCurrentTrajectoryLength).getPose();
	}

	/**
	 * Gets the translational error of the robot.
	 * 
	 * @return The translational error.
	 */
	public Translation2d getTranslationalError() {
		return new Translation2d(mError.getTranslation().x(), mError.getTranslation().y());
	}

	/**
	 * Gets the heading error of the robot.
	 * 
	 * @return The heading error.
	 */
	public Rotation2d getHeadingError() {
		return mError.getRotation();
	}

	/**
	 * Checks if the path is finished.
	 * 
	 * @return True if the path is finished, false otherwise.
	 */
	public boolean isPathFinished() {
		return mPathIsFinished;
	}

	/**
	 * Gets the current setpoint of the motion planner.
	 * 
	 * @return The current setpoint.
	 */
	public PPPathPointState getSetpoint() {
		return mSetpoint;
	}
}