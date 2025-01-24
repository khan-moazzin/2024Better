package com.team5817.frc2025;

import com.team5817.frc2025.subsystems.vision.VisionPoseAcceptor;
import com.team5817.lib.drivers.Subsystem;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.MovingAverageTwist2d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Map;
import java.util.Optional;

public class RobotState extends Subsystem{
	private static RobotState mInstance;

	public static RobotState getInstance() {
		if (mInstance == null) {
			mInstance = new RobotState();
		}
		return mInstance;
	}

	private static final int kObservationBufferSize = 50;
	private static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.05, 1), Math.pow(0.05, 1)); // drive
	private static final Matrix<N2, N1> kLocalMeasurementStdDevs = VecBuilder.fill(
			Math.pow(0.02, 1), // vision
			Math.pow(0.02, 1));
	private Optional<VisionUpdate> mLatestVisionUpdate;

	private Optional<Translation2d> initial_global_error = Optional.empty();
	private Optional<Translation2d> initial_specialized_pose = Optional.empty();
	private InterpolatingTreeMap<InterpolatingDouble, Pose2d> odometry_pose;
	private InterpolatingTreeMap<InterpolatingDouble, Translation2d> global_pose_error;
	private InterpolatingTreeMap<InterpolatingDouble, Translation2d> specialized_pose_error;
	private ExtendedKalmanFilter<N2, N2, N2> mGlobalKalmanFilter;
	private ExtendedKalmanFilter<N2, N2, N2> mSpecializedKalmanFilter;
	private VisionPoseAcceptor mPoseAcceptor;

	private Twist2d vehicle_velocity_measured;
	private Twist2d vehicle_velocity_predicted;
	private MovingAverageTwist2d vehicle_velocity_measured_filtered;

	private boolean mHasRecievedVisionUpdate = false;
	private boolean mIsInAuto = false;

	public RobotState() {
		reset(0.0, Pose2d.identity());
	}

	/**
	 * Clears pose history and sets odometry pose as truth.
	 *
	 * @param now                     Current timestamp.
	 * @param initial_odom_to_vehicle Initial pose of the robot.
	 */
	public synchronized void reset(double now, Pose2d initial_odom_to_vehicle) {
		odometry_pose = new InterpolatingTreeMap<>(kObservationBufferSize);
		odometry_pose.put(new InterpolatingDouble(now), initial_odom_to_vehicle);
		global_pose_error = new InterpolatingTreeMap<>(kObservationBufferSize);
		global_pose_error.put(new InterpolatingDouble(now), getInitialGlobalError());
		vehicle_velocity_measured = Twist2d.identity();
		vehicle_velocity_predicted = Twist2d.identity();
		vehicle_velocity_measured_filtered = new MovingAverageTwist2d(25);
		mLatestVisionUpdate = Optional.empty();
		mPoseAcceptor = new VisionPoseAcceptor();
	}

	/**
	 * Reconstructs Kalman Filter.
	 */
	public synchronized void resetKalman() {
		mGlobalKalmanFilter = new ExtendedKalmanFilter<N2, N2, N2>(
				Nat.N2(), // Dimensions of output (x, y)
				Nat.N2(), // Dimensions of predicted error shift (dx, dy) (always 0)
				Nat.N2(), // Dimensions of vision (x, y)
				(x, u) -> u, // The derivative of the output is predicted shift (always 0)
				(x, u) -> x, // The output is position (x, y)
				kStateStdDevs, // Standard deviation of position (uncertainty propagation with no vision)
				kLocalMeasurementStdDevs, // Standard deviation of vision measurements
				Constants.kLooperDt);
		mSpecializedKalmanFilter = new ExtendedKalmanFilter<N2, N2, N2>(
				Nat.N2(), // Dimensions of output (x, y)
				Nat.N2(), // Dimensions of predicted error shift (dx, dy) (always 0)
				Nat.N2(), // Dimensions of vision (x, y)
				(x, u) -> u, // The derivative of the output is predicted shift (always 0)
				(x, u) -> x, // The output is position (x, y)
				kStateStdDevs, // Standard deviation of position (uncertainty propagation with no vision)
				kLocalMeasurementStdDevs, // Standard deviation of vision measurements
				Constants.kLooperDt);
	}

	/**
	 * Adds new odometry pose update.
	 *
	 * @param now                Timestamp of observation.
	 * @param odometry_pose      Reported pose from odometry.
	 * @param measured_velocity  Measured field-relative velocity.
	 * @param predicted_velocity Predicted field-relative velocity (usually swerve
	 *                           setpoint).
	 */
	public synchronized void addOdometryUpdate(
			double now, Pose2d odometry_update, Twist2d measured_velocity, Twist2d predicted_velocity) {
		odometry_pose.put(new InterpolatingDouble(now), odometry_update);
		mGlobalKalmanFilter.predict(
				VecBuilder.fill(0.0, 0.0), Constants.kLooperDt); // Propagate error of current vision prediction
		mSpecializedKalmanFilter.predict(
				VecBuilder.fill(0.0, 0.0), Constants.kLooperDt); // Propagate error of current vision prediction
		vehicle_velocity_measured = measured_velocity;
		vehicle_velocity_measured_filtered.add(measured_velocity);
		vehicle_velocity_predicted = predicted_velocity;
	}

	/**
	 * Adds new vision pose update.
	 *
	 * @param update Info about vision update.
	 */
	public synchronized void addVisionUpdate(VisionUpdate update) {
		Translation2d field_to_vision = update.getFieldToVision();
		// If it's the first update don't do filtering
		if (!mLatestVisionUpdate.isPresent() || initial_global_error.isEmpty()) {
			double vision_timestamp = update.timestamp;
			Pose2d proximate_dt_pose = odometry_pose.getInterpolated(new InterpolatingDouble(vision_timestamp));	
			Translation2d odom_to_vehicle_translation = proximate_dt_pose.getTranslation();
			Translation2d field_to_odom = field_to_vision.translateBy(odom_to_vehicle_translation.inverse());
			global_pose_error.put(new InterpolatingDouble(vision_timestamp), field_to_odom);
			initial_global_error = Optional.of(global_pose_error.lastEntry().getValue());
			mGlobalKalmanFilter.setXhat(0, field_to_odom.x());
			mGlobalKalmanFilter.setXhat(1, field_to_odom.y());
			mLatestVisionUpdate = Optional.ofNullable(update);
		} else {
			double vision_timestamp = mLatestVisionUpdate.get().timestamp;
			Pose2d proximate_dt_pose = odometry_pose.getInterpolated(new InterpolatingDouble(vision_timestamp));
			mLatestVisionUpdate = Optional.ofNullable(update);
				if (mPoseAcceptor.shouldAcceptVision(
					vision_timestamp,
					new Pose2d(field_to_vision, new Rotation2d()),
					getLatestGlobalPose(),
					vehicle_velocity_measured,
					mIsInAuto)) {
				Translation2d field_to_odom = field_to_vision.translateBy(
						proximate_dt_pose.getTranslation().inverse());
				try {
					Vector<N2> stdevs = VecBuilder.fill(Math.pow(update.xy_stdev, 1), Math.pow(update.xy_stdev, 1));
					mGlobalKalmanFilter.correct(
							VecBuilder.fill(0.0, 0.0),
							VecBuilder.fill(
									field_to_odom.getTranslation().x(),
									field_to_odom.getTranslation().y()),
							StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), stdevs));
					global_pose_error.put(
							new InterpolatingDouble(vision_timestamp),
							new Translation2d(mGlobalKalmanFilter.getXhat(0), mGlobalKalmanFilter.getXhat(1)));
					if (!getHasRecievedVisionUpdate()) {
						mHasRecievedVisionUpdate = true;
					}
				} catch (Exception e) {
					DriverStation.reportError(update.xy_stdev + "//QR Decomposition failed: ", e.getStackTrace());
				}
			}
		}
	}

	public synchronized void addSpecializedVisionUpdate(VisionUpdate update) {
		Translation2d vision_pose = update.getFieldToVision();
		// If it's the first update don't do filtering
		if (!mLatestVisionUpdate.isPresent() || initial_specialized_pose.isEmpty()) {
			double vision_timestamp = update.timestamp;
			Pose2d proximate_dt_pose = odometry_pose.getInterpolated(new InterpolatingDouble(vision_timestamp));	
			Translation2d odometry_pose_translation = proximate_dt_pose.getTranslation();
			Translation2d field_to_odom = vision_pose.translateBy(odometry_pose_translation.inverse());
			specialized_pose_error.put(new InterpolatingDouble(vision_timestamp), field_to_odom);
			initial_specialized_pose = Optional.of(specialized_pose_error.lastEntry().getValue());
			mSpecializedKalmanFilter.setXhat(0, field_to_odom.x());
			mSpecializedKalmanFilter.setXhat(1, field_to_odom.y());
			mLatestVisionUpdate = Optional.ofNullable(update);
		} else {
			double vision_timestamp = mLatestVisionUpdate.get().timestamp;
			Pose2d current_odom_pose = odometry_pose.getInterpolated(new InterpolatingDouble(vision_timestamp));
			mLatestVisionUpdate = Optional.ofNullable(update);
				if (mPoseAcceptor.shouldAcceptVision(
					vision_timestamp,
					new Pose2d(vision_pose, new Rotation2d()),
					getLatestGlobalPose(),
					vehicle_velocity_measured,
					mIsInAuto)) {
				Translation2d pose_translational_error = vision_pose.translateBy(
						current_odom_pose.getTranslation().inverse());
				try {
					Vector<N2> stdevs = VecBuilder.fill(Math.pow(update.xy_stdev, 1), Math.pow(update.xy_stdev, 1));
					mSpecializedKalmanFilter.correct(
							VecBuilder.fill(0.0, 0.0),
							VecBuilder.fill(
									pose_translational_error.getTranslation().x(),
									pose_translational_error.getTranslation().y()),
							StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), stdevs));
					specialized_pose_error.put(
							new InterpolatingDouble(vision_timestamp),
							new Translation2d(mSpecializedKalmanFilter.getXhat(0), mSpecializedKalmanFilter.getXhat(1)));
					if (!getHasRecievedVisionUpdate()) {
						mHasRecievedVisionUpdate = true;
					}
				} catch (Exception e) {
					DriverStation.reportError(update.xy_stdev + "//QR Decomposition failed: ", e.getStackTrace());
				}
			}
		}
	}
	/**
	 * Gets initial odometry error. Odometry initializes to the origin, while the
	 * robot starts at an unknown position on the field.
	 *
	 * @return Initial odometry error translation.
	 */
	public synchronized Translation2d getInitialGlobalError() {
		if (initial_global_error.isEmpty()) return Translation2d.identity();
		return initial_global_error.get();
	}

	/**
	 * @return Latest field relative robot pose.
	 */
	public synchronized Pose2d getLatestGlobalPose() {
		Pose2d odometry_pose = getLatestOdomPose().getValue();

		Translation2d globalPoseError = getLatestGlobalError();
		return new Pose2d(globalPoseError.translateBy(odometry_pose.getTranslation()), odometry_pose.getRotation());
	}

	/**
	 * Gets field relative robot pose from history. Linearly interpolates between
	 * gaps.
	 *
	 * @param timestamp Timestamp to look up.
	 * @return Field relative robot pose at timestamp.
	 */
	public synchronized Pose2d getGlobalPose(double timestamp) {
		Pose2d odom_pose = getOdomPose(timestamp);

		Translation2d pose_error = getGlobalError(timestamp);
		return new Pose2d(pose_error.translateBy(odom_pose.getTranslation()), odom_pose.getRotation());
	}

	/**
	 * Gets interpolated robot pose using predicted robot velocity from latest
	 * odometry update.
	 *
	 * @param lookahead_time Scalar for predicted velocity.
	 * @return Predcited robot pose at lookahead time.
	 */
	public synchronized Pose2d getPredictedGlobalPose(double lookahead_time) {
		Pose2d odom_pose = getPredictedOdomPose(lookahead_time);

		Translation2d pose_error = getLatestGlobalError();
		return new Pose2d(pose_error.translateBy(odom_pose.getTranslation()), odom_pose.getRotation());
	}

	/**
	 * @return Latest odometry pose.
	 */
	public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestOdomPose() {
		return odometry_pose.lastEntry();
	}

	public synchronized Translation2d getInitialSpecializedError() {
		if (initial_global_error.isEmpty()) return Translation2d.identity();
		return initial_global_error.get();
	}

	/**
	 * @return Latest field relative robot pose.
	 */
	public synchronized Pose2d getLatestSpecializedPose() {
		Pose2d odometry_pose = getLatestOdomPose().getValue();

		Translation2d globalPoseError = getLatestGlobalError();
		return new Pose2d(globalPoseError.translateBy(odometry_pose.getTranslation()), odometry_pose.getRotation());
	}

	/**
	 * Gets field relative robot pose from history. Linearly interpolates between
	 * gaps.
	 *
	 * @param timestamp Timestamp to look up.
	 * @return Field relative robot pose at timestamp.
	 */
	public synchronized Pose2d getSpecializedPose(double timestamp) {
		Pose2d odom_pose = getOdomPose(timestamp);

		Translation2d pose_error = getGlobalError(timestamp);
		return new Pose2d(pose_error.translateBy(odom_pose.getTranslation()), odom_pose.getRotation());
	}

	/**
	 * Gets interpolated robot pose using predicted robot velocity from latest
	 * odometry update.
	 *
	 * @param lookahead_time Scalar for predicted velocity.
	 * @return Predcited robot pose at lookahead time.
	 */
	public synchronized Pose2d getPredictedSpecializedPose(double lookahead_time) {
		Pose2d odom_pose = getPredictedOdomPose(lookahead_time);

		Translation2d pose_error = getLatestGlobalError();
		return new Pose2d(pose_error.translateBy(odom_pose.getTranslation()), odom_pose.getRotation());
	}


	/**
	 * Gets odometry pose from history. Linearly interpolates between gaps.
	 *
	 * @param timestamp Timestamp to loop up.
	 * @return Odometry relative robot pose at timestamp.
	 */
	public synchronized Pose2d getOdomPose(double timestamp) {
		return odometry_pose.getInterpolated(new InterpolatingDouble(timestamp));
	}

	/**
	 * Gets interpolated odometry pose using predicted robot velocity from latest
	 * odometry update.
	 *
	 * @param lookahead_time Scalar for predicted velocity.
	 * @return Predcited odometry pose at lookahead time.
	 */
	public synchronized Pose2d getPredictedOdomPose(double lookahead_time) {
		return getLatestOdomPose()
				.getValue()
				.transformBy(Pose2d.exp(vehicle_velocity_predicted.scaled(lookahead_time)));
	}

	/**
	 * @return Latest odometry error translation.
	 */
	public synchronized Translation2d getLatestGlobalError() {
		return getGlobalError(global_pose_error.lastKey().value);
	}

	/**
	 * Gets odometry error translation at timestamp. Linearly interpolates between gaps.
	 * @param timestamp Timestamp to look up.
	 * @return Odometry error at timestamp.
	 */
	public synchronized Translation2d getGlobalError(double timestamp) {
		if (global_pose_error.isEmpty()) return Translation2d.identity();
		return global_pose_error.getInterpolated(new InterpolatingDouble(timestamp));
	}

	/**
	 * @return Predicted robot velocity from last odometry update.
	 */
	public synchronized Twist2d getPredictedVelocity() {
		return vehicle_velocity_predicted;
	}

	/**
	 * @return Measured robot velocity from last odometry update.
	 */
	public synchronized Twist2d getMeasuredVelocity() {
		return vehicle_velocity_measured;
	}

	/**
	 * @return Measured robot velocity smoothed using a moving average filter.
	 */
	public synchronized Twist2d getSmoothedVelocity() {
		return vehicle_velocity_measured_filtered.getAverage();
	}

	/**
	 * @return Gets if estimator has recieved a vision update.
	 */
	public synchronized boolean getHasRecievedVisionUpdate() {
		return mHasRecievedVisionUpdate;
	}

	/**
	 * Updates tracker to use stricter auto vision filtering.
	 * @param in_auto If auto filters should be used.
	 */
	public synchronized void setIsInAuto(boolean in_auto) {
		mIsInAuto = in_auto;
	}

	/**
	 * Class to hold information about a vision update.
	 */
	public static class VisionUpdate {
		private double timestamp;
		private Pose3d target_to_camera;
		private double ta;
		private Translation2d field_to_vision;
		private double xy_stdev;
		private int mID;

		public VisionUpdate(int id, double timestamp,double ta, Pose3d target_to_camera,Translation2d field_to_vision , double xy_stdev) {
			this.mID = id;
			this.ta = ta;			
			this.timestamp = timestamp;
			this.field_to_vision = field_to_vision;
			this.xy_stdev = xy_stdev;
		}

		public double getTimestamp() {
			return timestamp;
		}

		public Pose3d getTargetToCamera(){
			return target_to_camera;
		}

		public Translation2d getFieldToVision(){
			return field_to_vision;
		}
		
		public double getXYStdev() {
			return xy_stdev;
		}

		public double getTa() {
			return ta;
		}

		public int getID(){
			return mID;
		}
	}
}
