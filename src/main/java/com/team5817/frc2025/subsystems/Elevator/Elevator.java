package com.team5817.frc2025.subsystems.Elevator;

import com.team5817.frc2025.Robot;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.State.ServoState;
import com.team5817.lib.drivers.StateBasedServoMotorSubsystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import lombok.Getter;

import org.littletonrobotics.junction.Logger;

/**
 * Elevator subsystem for controlling the elevator mechanism.
 */
public class Elevator extends StateBasedServoMotorSubsystem<Elevator.State> {
	public static Elevator mInstance;

	/**
	 * Returns the singleton instance of the Elevator.
	 * 
	 * @return the singleton instance of the Elevator
	 */
	public static Elevator getInstance() {
		if (mInstance == null) {
			mInstance = new Elevator(ElevatorConstants.kElevatorServoConstants);
		}
		return mInstance;
	}

	private double distanceFromScoringPosition = 0;
	private double scoringOffset = 0;
	

	final static double kStrictError = .05;
	final static double kMediumError = .1;
	final static double kLenientError = .2;

	/**
	 * Enum representing the different states of the elevator.
	 */
	public enum State implements ServoState {
		L4(1.8635330123363545, kStrictError, ElevatorConstants.kHighOffsetMap),
		L3(1.211806509200769-.06, kStrictError, ElevatorConstants.kMidOffsetMap),
		L2(.754804+.03, kStrictError, ElevatorConstants.kMidOffsetMap),
		L1(0.219, kStrictError),
		A1(0.59, kMediumError),
		A2(1, kMediumError),
		NET(2.035, kStrictError),
		ZERO(0, kLenientError),
		PROCESS(0.0, kLenientError),
		CLEAR(.35,kStrictError),
		STOW(0.0, kStrictError);

		@Getter private double desiredPosition = 0;
		@Getter private double allowableError = 20;
		InterpolatingDoubleTreeMap map;

		
		State(double output, double allowable_error) {
			this(output, allowable_error, null);
		}
		State(double output, double allowable_error, InterpolatingDoubleTreeMap map) {
			this.desiredPosition = output;
			this.allowableError = allowable_error;
			this.map = map;
		}


		public double getTrackedOutput(double distanceFromScoringPosition){
			if(map == null){
				return desiredPosition;
			}
			double des = this.desiredPosition + map.get(distanceFromScoringPosition);

			des = Util.limit(des, ElevatorConstants.kElevatorServoConstants.kMinUnitsLimit, ElevatorConstants.kElevatorServoConstants.kMaxUnitsLimit);

			return des;
		}
		public boolean isDisabled(){
			return false;
		}
	}

	/**
	 * Constructs an Elevator with the given constants.
	 * 
	 * @param constants the constants for the elevator
	 */
	public Elevator(final ServoMotorSubsystemConstants constants) {
		super(constants,State.ZERO,false);
		enableSoftLimits(false);
	}

	/**
	 * Registers the enabled loops for the elevator.
	 * 
	 * @param enabledLooper the enabled looper
	 */
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
			}

			@Override
			public void onLoop(double timestamp) {
			}
		});
	}
	public void updateOnBranchDistance(double dist){
		this.distanceFromScoringPosition = dist;
	}
	public void setManualOffset(double offset){
		this.scoringOffset = offset;
	}
	public void changeManualOffset(double deltaOffset){
		this.scoringOffset+=deltaOffset;
	}


	@Override
	public void readPeriodicInputs() {
		super.readPeriodicInputs();
	}

	@Override
	public void writePeriodicOutputs() {
		double trackedOutput = mState.getTrackedOutput(distanceFromScoringPosition);
		if(mState==State.L1||mState==State.L2||mState==State.L3||mState==State.L4)
			trackedOutput+=scoringOffset;

		if (mControlState == ControlState.MOTION_MAGIC)
			setSetpointMotionMagic(trackedOutput);
		super.writePeriodicOutputs();
	}

	@Override
	public void outputTelemetry() {
		Pose3d current = new Pose3d(Math.cos(Units.degreesToRadians(84)) * mServoInputs.position_units, 0,
				Math.sin(Units.degreesToRadians(84)) * mServoInputs.position_units, new Rotation3d());

		Robot.mechPoses[1] = current.div(3);
		Robot.mechPoses[2] = current.div(3).times(2);
		Robot.mechPoses[3] = current;

		
		Logger.recordOutput("Elevator/Offset", this.scoringOffset);

		super.outputTelemetry();
	}
	public boolean getAtState(){
		return atState;
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

}