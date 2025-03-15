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

import java.util.concurrent.ConcurrentHashMap.KeySetView;

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
	
	private State mCurrentState = State.ZERO;
	private boolean atState = false;
	private double branchDist = 0;
	private double offset = 0;

	final static double kStrictError = 1;
	final static double kMediumError = 2 ;
	final static double kLenientError = 5;

	public enum State {

		L4(91.2, kStrictError,EndEffectorWristConstants.kHighOffsetMap),
		L3(24, kStrictError,EndEffectorWristConstants.kMidOffsetMap),
		L2(31.4, kStrictError,EndEffectorWristConstants.kMidOffsetMap),
		L1(0, kStrictError),
		A1(180.342, kMediumError),
		A2(180.342, kMediumError),
		NET(175.342, kMediumError),
		ZERO(0, kLenientError),
		INTAKING(175.342, kStrictError),
		PINCH(131, 70),
		STOW(175.342, kStrictError);

		double output = 0;
		double allowable_error = 0;
		boolean home = false;
		InterpolatingDoubleTreeMap map;

		State(double output, double allowable_error,InterpolatingDoubleTreeMap map ,boolean home) {
			this.output = output;
			this.allowable_error = allowable_error;
			this.map = map;
			this.home = home;
		}
		State(double output, double allowable_error){
			this(output,allowable_error,null,false);
		}
		State(double output,double allowable_error,boolean home){
			this(output,allowable_error,null,home);
		}
		State(double output, double allowable_error,InterpolatingDoubleTreeMap map){
			this(output,allowable_error,map,false);
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
			public void onLoop(double timestamp) {
				if (getSetpoint() == mConstants.kHomePosition  && mWantsHome && !mHoming) {
					setWantHome(true);
				} else if (mControlState != ControlState.OPEN_LOOP && mHoming) {
					setWantHome(false);
				}

			}

		});
	}
	public void conformToState(State state){
		mCurrentState = state;
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
		double trackedOutput = mCurrentState.getTrackedOutput(branchDist)+offset;
		trackedOutput = Util.limit(trackedOutput, mConstants.kMinUnitsLimit,mConstants.kMaxUnitsLimit);
		setSetpointMotionMagic(trackedOutput);
		atState = Util.epsilonEquals(getPosition(),trackedOutput, mCurrentState.allowable_error);
		super.writePeriodicOutputs();
	}

	@Override
	public void outputTelemetry() {
		Robot.mechPoses[5] = Robot.mechPoses[4]
				.transformBy(new Transform3d(new Translation3d(.221, 0, .278), new Rotation3d(Units.degreesToRadians(0),
						Units.degreesToRadians(180+14.252+mServoInputs.position_units), Units.degreesToRadians(0))));

		Robot.desMechPoses[5] = Robot.desMechPoses[4]
				.transformBy(new Transform3d(new Translation3d(.221, 0, .278), new Rotation3d(Units.degreesToRadians(0),
						Units.degreesToRadians(180+14.252+demand), Units.degreesToRadians(0))));
		Logger.recordOutput("EndEffectorWrist/Offset", this.offset);
		Logger.recordOutput(mConstants.kName+"/AtState", atState);
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
				mCurrentState = _wantedState;
				mWantsHome = _wantedState.home;
			}

			@Override
			public boolean isFinished() {
				return atState;
			}
		};
	}

}