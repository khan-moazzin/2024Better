package com.team5817.frc2025;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.MovingAverageTwist2d;
import com.team5817.frc2025.subsystems.vision.VisionPoseAcceptor;
import com.team5817.lib.util.UnscentedKalmanFilter;


public class RobotState {
    // Existing member variables
    private static RobotState mInstance;
    private boolean mIsInAuto = true;
    private Optional<VisionUpdate> mLatestVisionUpdate;
    private UnscentedKalmanFilter<N2, N2, N2> mKalmanFilter;
    private VisionPoseAcceptor mPoseAcceptor;
    private Pose2d mDisplayVisionPose;
    private Pose2d mSetpointPose;
    public Field2d mField2d;
    private boolean mHasBeenEnabled = false;

    // New variables for the specialized Kalman filter and vision update handling
    private UnscentedKalmanFilter<N2, N2, N2> mSpecializedKalmanFilter;
    private Optional<VisionUpdate> mLatestSpecializedVisionUpdate;
    private InterpolatingTreeMap<InterpolatingDouble, Translation2d> specializedVisionPoseComponent;
    private Optional<Translation2d> initialPoseErrorSpecialized = Optional.empty();  // Separate initial pose error for specialized updates

    private static final int kObservationBufferSize = 50;
    private Optional<Translation2d> initialPoseError = Optional.empty();
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> poseFromOdom;
    private InterpolatingTreeMap<InterpolatingDouble, Translation2d> visionPoseComponent;

    private Twist2d PredictedVelocity;
    private Twist2d MeasuredVelocity;
    private MovingAverageTwist2d filteredMeasuredVelocity;

    public static RobotState getInstance() {
        if (mInstance == null) 

            mInstance = new RobotState();
        return mInstance;
    }

    private RobotState() {
        reset(0.0, Pose2d.fromTranslation(new Translation2d(0, 0)));
    }

    public synchronized void reset(double start_time, Pose2d initialPose) {
        resetKalmanFilters();
        poseFromOdom = new InterpolatingTreeMap<>(kObservationBufferSize);
        poseFromOdom.put(new InterpolatingDouble(start_time), initialPose);
        visionPoseComponent = new InterpolatingTreeMap<>(kObservationBufferSize);
        visionPoseComponent.put(new InterpolatingDouble(start_time), Translation2d.identity());
        specializedVisionPoseComponent = new InterpolatingTreeMap<>(kObservationBufferSize);

        PredictedVelocity = Twist2d.identity();
        MeasuredVelocity = Twist2d.identity();
        filteredMeasuredVelocity = new MovingAverageTwist2d(25);
        mLatestVisionUpdate = Optional.empty();
        mDisplayVisionPose = Pose2d.identity();
        mSetpointPose = Pose2d.identity();
        mPoseAcceptor = new VisionPoseAcceptor();
        initialPoseError = Optional.empty();

        // Reset specialized Kalman filter and vision update tracker
        resetSpecializedKalmanFilters();
    }

    public synchronized void resetSpecializedKalmanFilters() {
        mSpecializedKalmanFilter = new UnscentedKalmanFilter<>(
            Nat.N2(),
            Nat.N2(),
            (x, u) -> VecBuilder.fill(0.0, 0.0),
            (x, u) -> x,
            Constants.kStateStdDevs,
            Constants.kLocalMeasurementStdDevs, .01
        );

        mLatestSpecializedVisionUpdate = Optional.empty();
        initialPoseErrorSpecialized = Optional.empty();  // Reset specialized initial pose error
    }

   public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

    public synchronized void resetKalmanFilters() {
       mKalmanFilter =
        new UnscentedKalmanFilter<>(
            Nat.N2(),
            Nat.N2(),
            (x, u) -> VecBuilder.fill(0.0, 0.0),
            (x, u) -> x,
            Constants.kStateStdDevs,
            Constants.kLocalMeasurementStdDevs, .01);

    }

    public synchronized boolean getHasBeenEnabled() {
        return mHasBeenEnabled;
    }

    public synchronized void setHasBeenEnabled(boolean hasBeenEnabled) {
        mHasBeenEnabled = hasBeenEnabled;
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getPoseFromOdom(double timestamp) {
        return poseFromOdom.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestPoseFromOdom() {
        return poseFromOdom.lastEntry();
    }

   public synchronized void addPoseObservation(double timestamp, Pose2d observation) {
        poseFromOdom.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addOdomObservations(double timestamp, Pose2d poseFromOdom, Twist2d measured_velocity, Twist2d predicted_velocity) {
            mKalmanFilter.predict(VecBuilder.fill(0.0, 0.0), .01);

              
        addPoseObservation(timestamp, poseFromOdom);
        MeasuredVelocity = measured_velocity;
        filteredMeasuredVelocity.add(MeasuredVelocity);
        PredictedVelocity = new Twist2d(predicted_velocity.dx, -predicted_velocity.dy, predicted_velocity.dtheta);
    }

    public synchronized Twist2d getPredictedVelocity() {
        return PredictedVelocity;
    }

    public synchronized Twist2d getMeasuredVelocity() {
        return MeasuredVelocity;
    }

    public synchronized Twist2d getSmoothedVelocity() {
        return filteredMeasuredVelocity.getAverage();
    }

    // New method to add specialized vision update with separate Kalman filter
   // Method to access the latest specialized vision update
    public synchronized Optional<VisionUpdate> getLatestSpecializedVisionUpdate() {
        return mLatestSpecializedVisionUpdate;
    }

    // Method to access the specialized Kalman filter state
    public synchronized Pose2d getSpecializedKalmanPose(double timestamp) {
        Pose2d poseFromOdom = getPoseFromOdom(timestamp);
        Translation2d kalmanPoseOffset = specializedVisionPoseComponent.getInterpolated(new InterpolatingDouble(timestamp));
        return new Pose2d(kalmanPoseOffset.translateBy(poseFromOdom.getTranslation()), poseFromOdom.getRotation());
    }

    public synchronized Pose2d getLatestSpecializedKalmanPose() {
        Pose2d poseFromOdom = getLatestPoseFromOdom().getValue();
        return new Pose2d(specializedVisionPoseComponent.lastEntry().getValue().add(poseFromOdom.getTranslation()), poseFromOdom.getRotation());
    }

    // Accessor for specialized initial pose error
    public synchronized Pose2d getInitialPoseErrorSpecialized() {
        if (initialPoseErrorSpecialized.isEmpty()) {
            return Pose2d.identity();
        }
        return Pose2d.fromTranslation(initialPoseErrorSpecialized.get());
    }


    /**
     * Adds a Vision Update
     * @param visionUpdate
     */
    public synchronized void addVisionUpdate(VisionUpdate visionUpdate) {
        // Use the same logic for both methods.
        if (!mLatestVisionUpdate.isEmpty() || initialPoseError.isPresent()) {
            // Get the Timestamp of the Vision Reading
            double visionTimestamp = mLatestVisionUpdate.get().getTimestamp();
            
            // Get pose from odometry based on vision timestamp
            Pose2d odomToVehicle = getPoseFromOdom(visionTimestamp);
            Pose2d visionFieldToVehicle = Pose2d.fromTranslation(visionUpdate.getFieldToVision());

            // Check if the vision update should be accepted
            if (!mPoseAcceptor.shouldAcceptVision(visionTimestamp, visionFieldToVehicle, getLatestGlobalKalmanPose(), MeasuredVelocity, false)) {
                return;
            }

            // Calculate the vision odometry error and apply it to the Kalman filter
            Translation2d visionOdomError = visionFieldToVehicle.getTranslation().translateBy(odomToVehicle.getTranslation().inverse());
            mDisplayVisionPose = visionFieldToVehicle;

            try {
                // Use the correct Kalman filter depending on the method
                mKalmanFilter.correct(VecBuilder.fill(0.0, 0.0), VecBuilder.fill(visionOdomError.x(), visionOdomError.y()));
                visionPoseComponent.put(new InterpolatingDouble(visionTimestamp), 
                    Pose2d.fromTranslation(new Translation2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1))).getTranslation());
            } catch (Exception e) {
                DriverStation.reportError("QR Decomposition failed: ", e.getStackTrace());
            }
        } else {
            // Handle case when there's no recent vision update or initial pose error
            double visionTimestamp = visionUpdate.timestamp;
            Pose2d proximatePose = poseFromOdom.getInterpolated(new InterpolatingDouble(visionTimestamp));

            Translation2d fieldToVision = visionUpdate.field_to_vision;
            Translation2d odomToVehicleTranslation = proximatePose.getTranslation();
            Translation2d fieldToOdom = fieldToVision.translateBy(odomToVehicleTranslation.inverse());

            visionPoseComponent.put(new InterpolatingDouble(visionTimestamp), fieldToOdom);
            initialPoseError = Optional.of(visionPoseComponent.lastEntry().getValue());

            mKalmanFilter.setXhat(0, fieldToOdom.x());
            mKalmanFilter.setXhat(1, fieldToOdom.y());
        }

        // Update the latest vision update
        mLatestVisionUpdate = Optional.ofNullable(visionUpdate);
    }

    public synchronized void addSpecializedVisionUpdate(VisionUpdate visionUpdate) {
        // Use the same logic as addVisionUpdate but with the specialized Kalman filter
        if (!mLatestSpecializedVisionUpdate.isEmpty() || initialPoseErrorSpecialized.isPresent()) {
            // Get the Timestamp of the Vision Reading
            double visionTimestamp = mLatestSpecializedVisionUpdate.get().getTimestamp();
            
            // Get pose from odometry based on vision timestamp
            Pose2d odomToVehicle = getPoseFromOdom(visionTimestamp);
            Pose2d visionFieldToVehicle = Pose2d.fromTranslation(visionUpdate.getFieldToVision());

            // Check if the vision update should be accepted
            if (!mPoseAcceptor.shouldAcceptVision(visionTimestamp, visionFieldToVehicle, getLatestSpecializedKalmanPose(), MeasuredVelocity, false)) {
                return;
            }

            // Calculate the vision odometry error and apply it to the specialized Kalman filter
            Translation2d visionOdomError = visionFieldToVehicle.getTranslation().translateBy(odomToVehicle.getTranslation().inverse());
            mDisplayVisionPose = visionFieldToVehicle;

            try {
                // Use the specialized Kalman filter
                mSpecializedKalmanFilter.correct(VecBuilder.fill(0.0, 0.0), VecBuilder.fill(visionOdomError.x(), visionOdomError.y()));
                specializedVisionPoseComponent.put(new InterpolatingDouble(visionTimestamp), 
                    Pose2d.fromTranslation(new Translation2d(mSpecializedKalmanFilter.getXhat(0), mSpecializedKalmanFilter.getXhat(1))).getTranslation());
            } catch (Exception e) {
                DriverStation.reportError("QR Decomposition failed: ", e.getStackTrace());
            }
        } else {
            // Handle case when there's no recent vision update or initial pose error
            double visionTimestamp = visionUpdate.timestamp;
            Pose2d proximatePose = poseFromOdom.getInterpolated(new InterpolatingDouble(visionTimestamp));

            Translation2d fieldToVision = visionUpdate.field_to_vision;
            Translation2d odomToVehicleTranslation = proximatePose.getTranslation();
            Translation2d fieldToOdom = fieldToVision.translateBy(odomToVehicleTranslation.inverse());

            specializedVisionPoseComponent.put(new InterpolatingDouble(visionTimestamp), fieldToOdom);
            initialPoseErrorSpecialized = Optional.of(specializedVisionPoseComponent.lastEntry().getValue());

            mSpecializedKalmanFilter.setXhat(0, fieldToOdom.x());
            mSpecializedKalmanFilter.setXhat(1, fieldToOdom.y());
        }

        // Update the latest specialized vision update
        mLatestSpecializedVisionUpdate = Optional.ofNullable(visionUpdate);
    }

    public synchronized Pose2d getDisplayVisionPose() {
        if (mDisplayVisionPose == null) {
            return Pose2d.fromTranslation(Translation2d.identity()); //Out of frame
        }
        return mDisplayVisionPose;
    }

    /**
     * Return Initial Vision Offset for Pure Odometry Visualization Purposes
     * @return
     */
    public synchronized Pose2d getInitialGlobalPoseError() {
        if (initialPoseError.isEmpty()) return Pose2d.identity();
        return Pose2d.fromTranslation(initialPoseError.get());
    }

    public synchronized Translation2d getGlobalVisionPoseComponent(double timestamp) {
        if (initialPoseError.isEmpty()) return Translation2d.identity();
        return initialPoseError.get().inverse().translateBy(visionPoseComponent.getInterpolated(new InterpolatingDouble(timestamp)));
    }

    public synchronized Translation2d getGlobalAbsoluteVisionPoseComponent(double timestamp) {
        return visionPoseComponent.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Translation2d getLatestGlobalVisionPoseComponent() {
        return getGlobalAbsoluteVisionPoseComponent(visionPoseComponent.lastKey().value);
    }
    /**
     * Get Current Field to Vehicle using Filter Idea 1 (Offset Space) => Add the Offset outputted by the Filter to Current Odom
     * @param timestamp
     * @return
     */
    public synchronized Pose2d getGlobalKalmanPose(double timestamp) {
        Pose2d poseFromOdom = getPoseFromOdom(timestamp);

        Translation2d kalmanPoseOffset = getGlobalAbsoluteVisionPoseComponent(timestamp);
        return new Pose2d(kalmanPoseOffset.translateBy(poseFromOdom.getTranslation()), poseFromOdom.getRotation());

    }


    public synchronized Pose2d getLatestGlobalKalmanPose() {
        Pose2d poseFromOdom = getLatestPoseFromOdom().getValue();
        return new Pose2d(getLatestGlobalVisionPoseComponent().getTranslation().add(poseFromOdom.getTranslation()), poseFromOdom.getRotation());
    }

	public void setIsAuto(boolean newAuto){
		this.mIsInAuto = newAuto;
	}


    public synchronized Optional<VisionUpdate> getLatestGlobalVisionUpdate() {
        return mLatestVisionUpdate;
    }

    public void setDisplaySetpointPose(Pose2d setpoint) {
        mSetpointPose = setpoint;
    }

	public static class VisionUpdate {
		private double timestamp;
		private double ta;
		private Translation2d field_to_vision;
		private int mID;

		public VisionUpdate(int id, double timestamp,double ta,Translation2d field_to_vision) {
			this.mID = id;
			this.ta = ta;			
			this.timestamp = timestamp;
			this.field_to_vision = field_to_vision;
		}

		public double getTimestamp() {
			return timestamp;
		}

        public Translation2d getFieldToVision(){
			return field_to_vision;
		}
		
		public double getTa() {
			return ta;
		}

		public int getID(){
			return mID;
		}
	}
}