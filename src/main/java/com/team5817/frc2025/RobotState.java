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
    private static RobotState mInstance;
	private boolean mIsInAuto = true;
    private Optional<VisionUpdate> mLatestVisionUpdate;
    private UnscentedKalmanFilter<N2, N2, N2> mKalmanFilter;
    private VisionPoseAcceptor mPoseAcceptor;
    private Pose2d mDisplayVisionPose;
    private Pose2d mSetpointPose;

    public Field2d mField2d;

    private boolean mHasBeenEnabled = false;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    private static final int kObservationBufferSize = 50;
    
    private Optional<Translation2d> initialPose = Optional.empty();
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> poseFromOdom;
    private InterpolatingTreeMap<InterpolatingDouble, Translation2d> visionPoseComponent;

    private Twist2d PredictedVelocity;
    private Twist2d MeasuredVelocity;
    private MovingAverageTwist2d filteredMeasuredVelocity;

    private RobotState() {
        reset(0.0, Pose2d.fromTranslation(new Translation2d(0,0)));
    }


    public synchronized void reset(double start_time, Pose2d initialPose) {
        resetKalmanFilters(start_time);
        poseFromOdom = new InterpolatingTreeMap<>(kObservationBufferSize);
        poseFromOdom.put(new InterpolatingDouble(start_time), initialPose);
        visionPoseComponent = new InterpolatingTreeMap<>(kObservationBufferSize);
        visionPoseComponent.put(new InterpolatingDouble(start_time), Translation2d.identity());
        PredictedVelocity = Twist2d.identity();
        MeasuredVelocity = Twist2d.identity();
        filteredMeasuredVelocity = new MovingAverageTwist2d(25);
        mLatestVisionUpdate = Optional.empty();
        mDisplayVisionPose = Pose2d.identity();
        mSetpointPose = Pose2d.identity();
        mPoseAcceptor = new VisionPoseAcceptor();
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

    public synchronized void resetKalmanFilters(double timestamp) {
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


    /**
     * Adds a Vision Update
     * @param visionUpdate
     */
    public synchronized void addVisionUpdate(VisionUpdate visionUpdate) {
        mLatestVisionUpdate = Optional.ofNullable(visionUpdate);
        if (!mLatestVisionUpdate.isEmpty()) {
            //Get the Timestamp of the Vision Reading
            double visionTimestamp = mLatestVisionUpdate.get().getTimestamp();
            
            Pose2d odomToVehicle = getPoseFromOdom(visionTimestamp);

            Pose2d visionFieldToVehicle = Pose2d.fromTranslation(visionUpdate.getFieldToVision());

            if (!mPoseAcceptor.shouldAcceptVision(mLatestVisionUpdate.get().getTimestamp(), visionFieldToVehicle, getLatestKalmanPose(),MeasuredVelocity, false)) {
                return;
            }
            

                var visionOdomError = visionFieldToVehicle.getTranslation().translateBy(odomToVehicle.getTranslation().inverse());
                mDisplayVisionPose = visionFieldToVehicle;
                try {
                    mKalmanFilter.correct(VecBuilder.fill(0.0, 0.0), VecBuilder.fill(visionOdomError.getTranslation().x(), visionOdomError.getTranslation().y()));
                    visionPoseComponent.put(new InterpolatingDouble(visionTimestamp), Pose2d.fromTranslation(new Translation2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1))).getTranslation());
                } catch (Exception e) {
                    DriverStation.reportError("QR Decomposition failed: ", e.getStackTrace());
                }
            } else {
                mDisplayVisionPose = null;
            }
        
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
    public synchronized Pose2d getInitialPose() {
        if (initialPose.isEmpty()) return Pose2d.identity();
        return Pose2d.fromTranslation(initialPose.get());
    }

    public synchronized Translation2d getVisionPoseComponent(double timestamp) {
        if (initialPose.isEmpty()) return Translation2d.identity();
        return initialPose.get().inverse().translateBy(visionPoseComponent.getInterpolated(new InterpolatingDouble(timestamp)));
    }

    public synchronized Translation2d getAbsoluteVisionPoseComponent(double timestamp) {
        return visionPoseComponent.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Translation2d getLatestVisionPoseComponent() {
        return getAbsoluteVisionPoseComponent(visionPoseComponent.lastKey().value);
    }
    /**
     * Get Current Field to Vehicle using Filter Idea 1 (Offset Space) => Add the Offset outputted by the Filter to Current Odom
     * @param timestamp
     * @return
     */
    public synchronized Pose2d getKalmanPose(double timestamp) {
        Pose2d poseFromOdom = getPoseFromOdom(timestamp);

        Translation2d kalmanPoseOffset = getAbsoluteVisionPoseComponent(timestamp);
        return new Pose2d(kalmanPoseOffset.translateBy(poseFromOdom.getTranslation()), poseFromOdom.getRotation());

    }

    public synchronized Pose2d getAbosoluteKalmanPose(double timestamp) {
        var initialPose_ = initialPose.orElse(Translation2d.identity());
        return Pose2d.fromTranslation(initialPose_).transformBy(getKalmanPose(timestamp));
    }

    public synchronized Pose2d getLatestKalmanPose() {
        Pose2d poseFromOdom = getLatestPoseFromOdom().getValue();
        return new Pose2d(getLatestVisionPoseComponent().getTranslation().add(poseFromOdom.getTranslation()), poseFromOdom.getRotation());
    }

	public void setIsAuto(boolean newAuto){
		this.mIsInAuto = newAuto;
	}


    public synchronized Optional<VisionUpdate> getLatestVisionUpdate() {
        return mLatestVisionUpdate;
    }

    public void outputTelemetry() {
        // Logger.recordOutput("RobotState/Robot Velocity", getMeasuredVelocity().toString());
        // Logger.recordOutput("RobotState/PoseFromOdometry",  new Pose2d(getLatestPoseFromOdom().getValue().getTranslation(), getLatestPoseFromOdom().getValue().getRotation().inverse()).wpi());
        // Logger.recordOutput("RobotState/Vision Pose Component", getAbsoluteVisionPoseComponent(Timer.getFPGATimestamp()).wpi());
        // Logger.recordOutput("RobotState/Filtered Pose", new Pose2d(getLatestKalmanPose().getTranslation(), getLatestKalmanPose().getRotation().inverse()).wpi());
        // Logger.recordOutput("RobotState/SetPoint Pose", mSetpointPose.wpi());
        // Logger.recordOutput("RobotState/Vision Pose", getDisplayVisionPose().wpi());
   }

    public void setDisplaySetpointPose(Pose2d setpoint) {
        mSetpointPose = setpoint;
    }
	public static class VisionUpdate {
		private double timestamp;
		private Pose3d target_to_camera;
		private double ta;
		private Translation2d field_to_vision;
		private int mID;

		public VisionUpdate(int id, double timestamp,double ta, Pose3d target_to_camera,Translation2d field_to_vision) {
			this.mID = id;
			this.ta = ta;			
			this.timestamp = timestamp;
			this.field_to_vision = field_to_vision;
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
		
		public double getTa() {
			return ta;
		}

		public int getID(){
			return mID;
		}
	}
}