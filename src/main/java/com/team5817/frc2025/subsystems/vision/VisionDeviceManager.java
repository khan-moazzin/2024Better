package com.team5817.frc2025.subsystems.vision;

import com.team5817.frc2025.Constants;
import com.team5817.frc2025.Robot;
import com.team5817.frc2025.RobotState;
import com.team5817.frc2025.Constants.PoseEstimatorConstants;
import com.team5817.frc2025.RobotState.VisionUpdate;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.lib.drivers.Subsystem;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Translation3d;
import com.team254.lib.util.MovingAverage;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

import org.littletonrobotics.junction.Logger;

public class VisionDeviceManager extends Subsystem {

	public static VisionDeviceManager mInstance;
	public static VisionDeviceManager getInstance() {
		if (mInstance == null) {
			mInstance = new VisionDeviceManager();
		}
		return mInstance;
	}

	private VisionDevice mDomCamera;
	private VisionDevice mSubCamera;
	private RobotState mRobotState;

	private List<VisionDevice> mAllCameras;

	private static double timestampOffset = 0.1;

	private MovingAverage mHeadingAvg = new MovingAverage(100);
	private double mMovingAvgRead = 0.0;

	private static boolean disable_vision = false;
	double timeOfLastUpdate = Double.MIN_VALUE;

	public VisionDeviceManager() {
		mDomCamera = new VisionDevice("limelight-right");
		mSubCamera = new VisionDevice("limelight-left");
		mAllCameras = List.of(mDomCamera, mSubCamera);
		mRobotState = RobotState.getInstance();
	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
			}

			@Override
			public void onLoop(double timestamp) {

			}

			@Override
			public void onStop(double timestamp) {
			}

		});
	}

	@Override
	public void readPeriodicInputs() {
		mMovingAvgRead = mHeadingAvg.getAverage();
		if ( Constants.mode == Constants.Mode.SIM) {
			RobotState.getInstance().addVisionUpdate(new VisionUpdate(1, Timer.getTimestamp(), 1.0,
					RobotState.getInstance().getPoseFromOdom(Timer.getTimestamp()).getTranslation()));
		} else {

			for (VisionDevice device : mAllCameras) {
				device.update(Timer.getTimestamp());
				mHeadingAvg.addNumber(device.getEstimatedHeading());
				if (!device.getVisionUpdate().isEmpty()) {
					VisionUpdate update = device.getVisionUpdate().get();
					RobotState.getInstance().addVisionUpdate(update);

					if (DriverStation.getAlliance().get() == Alliance.Red) {
						if (PoseEstimatorConstants.redTagIDFilters.contains(update.getID())||DriverStation.isDisabled()) {
							mRobotState.addSpecializedVisionUpdate(update);
						}

					} else {
						if (PoseEstimatorConstants.blueTagIDFilters.contains(update.getID())||DriverStation.isDisabled()) {
							mRobotState.addSpecializedVisionUpdate(update);
						}
					}
					if (update.getTimestamp() > timeOfLastUpdate)
						timeOfLastUpdate = update.getTimestamp();
				}
			}
		}
	}

	@Override
	public void writePeriodicOutputs() {
	}

	@Override
	public void outputTelemetry() {
		Logger.recordOutput("Elastic/Time Since Last Update", Timer.getTimestamp() - timeOfLastUpdate);
		SmartDashboard.putNumber("Vision heading moving avg", getMovingAverageRead());
		SmartDashboard.putBoolean("vision disabled", visionDisabled());
		for (VisionDevice device : mAllCameras) {
			device.outputTelemetry();
		}
	}

	public VisionDevice getBestDevice() {
		if (mDomCamera.getVisionUpdate().get().getTa() > mSubCamera.getVisionUpdate().get().getTa())
			return mDomCamera;
		return mSubCamera;
	}

	public boolean epipolarVerification(List<Translation2d> pointsCam1, List<Translation2d> pointsCam2) {

		if (pointsCam1.size() != pointsCam2.size()) {
			throw new IllegalArgumentException("Point lists must have the same size.");
		}
		double threshold = 1e-6; // Tolerance for numerical errors
		for (int i = 0; i < pointsCam1.size(); i++) {
			Translation3d x1 = toHomogeneous(pointsCam1.get(i));
			Translation3d x2 = toHomogeneous(pointsCam2.get(i));
			// Compute Fx1 = F * x1
			Translation3d Fx1 = multiplyMatrixVector(Constants.fundamentalMatrix, x1);
			// Compute x2 • (Fx1)
			double result = x2.dot(Fx1);
			// Check if result is close to zero
			if (Math.abs(result) > threshold) {
				return false;
			}
		}
		return true;

	}

	public boolean translationalFilter(Pose3d domTargetToCamera, Pose3d subDevice) {
		Transform3d expectedDelta = PoseEstimatorConstants.kDomVisionDevice.kRobotToCamera
				.plus(PoseEstimatorConstants.kSubVisionDevice.kRobotToCamera.inverse());
		Transform3d delta;
		delta = new Transform3d(domTargetToCamera, subDevice);
		Transform3d error = delta.plus(expectedDelta.inverse());
		return (error.getTranslation().getNorm() > 0.1 || error.getRotation().getAngle() > 0.5);// TODO Find threshold
																								// (meters and radians)

	}

	public Double getMovingAverageRead() {
		return mMovingAvgRead;
	}

	public MovingAverage getMovingAverage() {
		return mHeadingAvg;
	}

	public VisionDevice getLeftVision() {
		return mDomCamera;
	}

	public VisionDevice getRightVision() {
		return mSubCamera;
	}

	public static double getTimestampOffset() {
		return timestampOffset;
	}

	public static boolean visionDisabled() {
		return disable_vision;
	}

	public static void setDisableVision(boolean disable) {
		disable_vision = disable;
	}

	public boolean fullyConnected() {
		return false;
	}

	public static boolean verifyEpipolarGeometry(List<Translation2d> pointsCam1,
			List<Translation2d> pointsCam2,
			double[][] fundamentalMatrix) {
		if (pointsCam1.size() != pointsCam2.size()) {
			throw new IllegalArgumentException("Point lists must have the same size.");
		}

		double threshold = 1e-6; // Tolerance for numerical errors
		for (int i = 0; i < pointsCam1.size(); i++) {
			Translation3d x1 = toHomogeneous(pointsCam1.get(i));
			Translation3d x2 = toHomogeneous(pointsCam2.get(i));

			// Compute Fx1 = F * x1
			Translation3d Fx1 = multiplyMatrixVector(fundamentalMatrix, x1);

			// Compute x2 • (Fx1)
			double result = x2.dot(Fx1);

			// Check if result is close to zero
			if (Math.abs(result) > threshold) {
				return false;
			}
		}

		return true;
	}

	/**
	 * Converts a Translation2D to homogeneous coordinates (Translation3D).
	 */
	private static Translation3d toHomogeneous(Translation2d point) {
		return new Translation3d(point.x(), point.y(), 1.0);
	}

	/**
	 * Multiplies a 3x3 matrix with a 3d vector.
	 */
	private static Translation3d multiplyMatrixVector(double[][] matrix, Translation3d vector) {
		double x = matrix[0][0] * vector.x() + matrix[0][1] * vector.y() + matrix[0][2] * vector.z();
		double y = matrix[1][0] * vector.x() + matrix[1][1] * vector.y() + matrix[1][2] * vector.z();
		double z = matrix[2][0] * vector.x() + matrix[2][1] * vector.y() + matrix[2][2] * vector.z();
		return new Translation3d(x, y, z);
	}

}
