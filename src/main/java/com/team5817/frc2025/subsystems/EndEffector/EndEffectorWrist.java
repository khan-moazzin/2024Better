package com.team5817.frc2025.subsystems.EndEffector;

import com.team5817.frc2025.Robot;
import com.team5817.frc2025.Constants.EndEffectorWristConstants;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.ServoMotorSubsystem;
import com.team5817.lib.requests.Request;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

import org.littletonrobotics.junction.Logger;

public class EndEffectorWrist extends ServoMotorSubsystem{

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
	
	private State mState = State.ZERO;
	private boolean atState = false;
	private double branchDist = 0;
	private double offset = 0;

	final static double kStrictError = 1.5;
	final static double kMediumError = 2 ;
	final static double kLenientError = 2.5;

	public enum State {

		L4(71.89453125-89, kStrictError,EndEffectorWristConstants.kHighOffsetMap),
		L3(-45, kStrictError,EndEffectorWristConstants.kMidOffsetMap),
		L2(-47, kStrictError,EndEffectorWristConstants.kMidOffsetMap),
		L1(0-89, kStrictError),
		A1(180.342-89, kMediumError),
		A2(180.342-89, kMediumError),
		NET(175.342-89, kMediumError),
		ZERO(0-89, kMediumError),
		INTAKING(88.17 ,kStrictError),
		PINCH(131-89, 70),
		STOW(88.17 ,kStrictError);

		double output = 0;
		double allowable_error = 0;
		InterpolatingDoubleTreeMap map;

		State(double output, double allowable_error,InterpolatingDoubleTreeMap map) {
			this.output = output;
			this.allowable_error = allowable_error;
			this.map = map;
		}
		State(double output, double allowable_error){
			this(output,allowable_error,null);
		}
		
		public double getTrackedOutput(double position){
			if(map == null){
				return output;
			}
			double des = this.output + map.get(position);
			return des;
		}
	}

	/**
	 * Constructs an EndEffectorWrist with the given constants.
	 * 
	 * @param constants The servo motor subsystem constants.
	 * @param encoder_constants The absolute encoder constants.
	 */
	public EndEffectorWrist(final ServoMotorSubsystemConstants constants) {
		super(constants);

		enableSoftLimits(false);
		mMain.setPosition(0);
		// setSetpointMotionMagic(State.STOW.output);
	}

	/**
	 * Registers the enabled loops.
	 * 
	 * @param enabledLooper The enabled looper.
	 */
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
			}

			@Override
			public void onLoop(double timestamp) {}

		});
	}
	public void conformToState(State state){
		mState = state;
	}
	public void updateBranchDistance(double dist){
		branchDist = dist;
	}
	public void setManualOffset(double offset){
		this.offset = offset;
	}
	public void changeManualOffset(Double deltaOffset){
		this.offset+=deltaOffset;
	}

	@Override
	public void readPeriodicInputs() {
		super.readPeriodicInputs();

		Logger.processInputs("EndEffectorWrist", mServoInputs);
	}

	@Override
	public void writePeriodicOutputs() {
		double trackedOutput = mState.getTrackedOutput(branchDist);
		if(mState==State.L1||mState==State.L2||mState==State.L3||mState==State.L4)
			trackedOutput+=offset;
		trackedOutput = Util.limit(trackedOutput, mConstants.kMinUnitsLimit,mConstants.kMaxUnitsLimit);
		setSetpointMotionMagic(trackedOutput);
		atState = Util.epsilonEquals(getPosition(),trackedOutput, mState.allowable_error);
		super.writePeriodicOutputs();
	}

	@Override
	public void outputTelemetry() {
		Robot.mechPoses[4] = Robot.mechPoses[3]
				.transformBy(new Transform3d(new Translation3d(.22, 0, .2922), new Rotation3d(Units.degreesToRadians(0),
						Units.degreesToRadians(mServoInputs.position_units), Units.degreesToRadians(0))));

		Robot.desMechPoses[4] = Robot.desMechPoses[3]
				.transformBy(new Transform3d(new Translation3d(.22, 0, .2922), new Rotation3d(Units.degreesToRadians(0),
						Units.degreesToRadians(demand), Units.degreesToRadians(0))));
		Logger.recordOutput("EndEffectorWrist/Offset", this.offset);
		Logger.recordOutput(mConstants.kName+"/AtState", atState);
		Logger.recordOutput(mConstants.kName+"/State",	mState);
		super.outputTelemetry();
	}

	@Override
	public void stop() {
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	/**
	 * Returns a request to set the end effector wrist to the given state.
	 * 
	 * @param _wantedState The desired state.
	 * @return The state request.
	 */
	public Request stateRequest(State _wantedState) {
		return new Request() {
			@Override
			public void act() {
				mState = _wantedState;
			}

			@Override
			public boolean isFinished() {
				return atState;
			}
		};
	}

}