package com.team5817.frc2025.subsystems.EndEffector;

import com.team5817.frc2025.Robot;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorConstants.EndEffectorWristConstants;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.State.ServoState;
import com.team5817.lib.drivers.StateBasedServoMotorSubsystem;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import lombok.Getter;

import org.littletonrobotics.junction.Logger;

public class EndEffectorWrist extends StateBasedServoMotorSubsystem<EndEffectorWrist.State> {

	/**
	 * Singleton instance of the EndEffectorWrist.
	 */
	public static EndEffectorWrist mInstance;

	/**
	 * Returns the singleton instance of the EndEffectorWrist.
	 * 
	 * @return The instance of EndEffectorWrist.
	 */
	public static EndEffectorWrist getInstance() {
		if (mInstance == null) {
			mInstance = new EndEffectorWrist(EndEffectorWristConstants.kWristServoConstants);
		}
		return mInstance;
	}
	
	private double distanceFromScoringLocation = 0;
	private double scoringOffset = 0;

	final static double kStrictError = 1.5;
	final static double kMediumError = 2 ;
	final static double kLenientError = 2.5;

	public enum State implements ServoState {

		L4(71.89453125-89, kStrictError,EndEffectorWristConstants.kHighOffsetMap),
		L3(-31.185546874999993, kStrictError,EndEffectorWristConstants.kMidOffsetMap),
		L2(-40.185546874999993, kStrictError,EndEffectorWristConstants.kMidOffsetMap),
		L1(0-89, kStrictError),
		A1(180.342-89, kMediumError),
		A2(180.342-89, kMediumError),
		NET(175.342-89, kMediumError),
		ZERO(0-89, kMediumError),
		INTAKING(88.17 ,kStrictError),
		PINCH(131-89, 70),
		STOW(88.17 ,kStrictError);

		@Getter private double desiredPosition = 0;
		@Getter private double allowableError = 0;
		InterpolatingDoubleTreeMap map;

		State(double output, double allowable_error,InterpolatingDoubleTreeMap map) {
			this.desiredPosition = output;
			this.allowableError = allowable_error;
			this.map = map;
		}
		State(double output, double allowable_error){
			this(output,allowable_error,null);
		}
		
		public double getTrackedOutput(double distanceFromScoringPosition){
			if(map == null){
				return desiredPosition;
			}
			double des = this.desiredPosition + map.get(distanceFromScoringPosition);
			des = Util.limit(des, EndEffectorWristConstants.kWristServoConstants.kMinUnitsLimit,EndEffectorWristConstants.kWristServoConstants.kMaxUnitsLimit);

			return des;
		}
		
		public boolean isDisabled(){
			return false;
		}
	}

	/**
	 * Constructs an EndEffectorWrist with the given constants.
	 * 
	 * @param constants The servo motor subsystem constants.
	 * @param encoder_constants The absolute encoder constants.
	 */
	public EndEffectorWrist(final ServoMotorSubsystemConstants constants) {
		super(constants,State.ZERO,false);

		enableSoftLimits(false);
	}

	public void updateOnBranchDistance(double dist){
		distanceFromScoringLocation = dist;
	}

	public void setManualOffset(double offset){
		this.scoringOffset = offset;
	}

	public void changeManualOffset(Double deltaOffset){
		this.scoringOffset+=deltaOffset;
	}

	@Override
	public void writePeriodicOutputs() {
		double trackedOutput = mState.getTrackedOutput(distanceFromScoringLocation);
		if(mState==State.L1||mState==State.L2||mState==State.L3||mState==State.L4)
			trackedOutput+=scoringOffset;

		setSetpointMotionMagic(trackedOutput);
		
		super.writePeriodicOutputs();
	}

	@Override
	public void outputTelemetry() {
		Robot.mechPoses[4] = Robot.mechPoses[3]
				.transformBy(new Transform3d(new Translation3d(.22, 0, .2922), new Rotation3d(Units.degreesToRadians(0),
						Units.degreesToRadians(mServoInputs.position_units+89), Units.degreesToRadians(0))));

		Logger.recordOutput("EndEffectorWrist/Offset", this.scoringOffset);

		super.outputTelemetry();
	}
}