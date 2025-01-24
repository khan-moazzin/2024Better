package com.team5817.frc2025.subsystems.vision;

import com.team5817.frc2025.RobotState;
import com.team5817.frc2025.RobotState.VisionUpdate;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.frc2025.subsystems.vision.LimelightHelpers.PoseEstimate;
import com.team5817.lib.drivers.Pigeon;
import com.team5817.lib.drivers.Subsystem;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.util.MovingAverage;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.Optional;

public class VisionDevice extends Subsystem {
	private PeriodicIO mPeriodicIO = new PeriodicIO();

	private String mName;
	private NetworkTable mOutputTable;
	private MovingAverage mHeadingAverage = new MovingAverage(100);

	public VisionDevice(String name) {
		this.mName = name;
	
		mOutputTable = NetworkTableInstance.getDefault().getTable(name);
	}
	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				mHeadingAverage.clear();
			}

			@Override
			public void onLoop(double timestamp) {

			if (mPeriodicIO.seesTarget && mPeriodicIO.is_connected) {
				final double realTime = timestamp - mPeriodicIO.latency;
				mPeriodicIO.fps = mOutputTable.getEntry("fps").getInteger(0);
				mPeriodicIO.latency = mOutputTable.getEntry("latency").getDouble(0.0);
				mPeriodicIO.tagId = mOutputTable.getEntry("tid").getNumber(-1).intValue();
				mPeriodicIO.seesTarget = mOutputTable.getEntry("tv").getBoolean(false);
					
				mPeriodicIO.mt1Pose = new Pose2d(LimelightHelpers.getBotPose2d_wpiBlue(mName));
				PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(mName);
				mPeriodicIO.targetToCamera = LimelightHelpers.getTargetPose3d_CameraSpace(mName);
				mPeriodicIO.tagCounts = poseEstimate.tagCount;
				mPeriodicIO.mt2Pose = new Pose2d(poseEstimate.pose);
				VisionUpdate visionUpdate = new VisionUpdate(mPeriodicIO.tagId, timestamp,mPeriodicIO.ta, mPeriodicIO.targetToCamera, mPeriodicIO.mt2Pose.getTranslation(), realTime);
				mPeriodicIO.visionUpdate = Optional.of(visionUpdate);
				}

				LimelightHelpers.SetRobotOrientation(mName, Pigeon.getInstance().getYaw().getDegrees(), 0, 0, 0, 0, 0);

			}

			@Override
			public void onStop(double timestamp) {
				mHeadingAverage.clear();
			}
			});
		}

	public boolean movingAverageReady(){
		return mHeadingAverage.getSize() == 100;
	}

	public void addHeadingObservation(Rotation2d heading) {
		double degrees = heading.getDegrees();
		if(degrees < 0){
			degrees += 360;
		}
		mHeadingAverage.addNumber(degrees);
	}

	public double getEstimatedHeading(){
		return mHeadingAverage.getAverage();
	}

	@Override
	public void readPeriodicInputs() {



	}

	@Override
	public void outputTelemetry() {
	}

	@Override
	public void writePeriodicOutputs() {

	}
	public Optional<VisionUpdate> getVisionUpdate(){
		return mPeriodicIO.visionUpdate;
	}

	public static class PeriodicIO {

	// inputs
		
		double camera_exposure = 20;
		boolean camera_auto_exposure = false;
		double camera_gain = 10;

		// Outputs
		public long hb = 0;
		public long fps = -1;
		public boolean is_connected;
		public double latency = 0;
		public int tagId = 0;
		public boolean seesTarget = false;
		public Optional<VisionUpdate> visionUpdate = Optional.empty();
		public double ta = 0;
		public boolean useVision = true;
		public double tagCounts = 0;
		public Pose2d mt2Pose = new Pose2d();
		public Pose2d mt1Pose = new Pose2d();
		public Pose2d specializedPose = new Pose2d();
		public Pose3d targetToCamera = new Pose3d();
	}
}
