package com.team5817.frc2025.subsystems.EndEffector;

import com.team5817.frc2025.Robot;
import com.team254.lib.util.DelayedBoolean;
import com.team5817.frc2025.Constants;
import com.team5817.frc2025.Constants.EndEffectorWristConstants;
import com.team5817.frc2025.Constants.IntakeDeployConstants;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.ServoMotorSubsystem;
import com.team5817.lib.requests.Request;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.DoubleBinaryOperator;

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
	

	final static double kStrictError = 1;
	final static double kMediumError = 2 ;
	final static double kLenientError = 5;

	public enum State {

		L4(81.2, kStrictError),
		L3(0, kStrictError),
		L2(15, kStrictError),
		L1(0, kStrictError),
		A1(207, kMediumError),
		A2(207, kMediumError),
		NET(207, kMediumError),
		ZERO(0, kLenientError,true),
		INTAKING(164, kStrictError),
		STOW(168
		, kStrictError);

		double output = 0;
		double allowable_error = 0;
		boolean home = false;

		State(double output, double allowable_error,boolean home) {
			this.output = output;
			this.allowable_error = allowable_error;
			this.home = home;
		}
		State(double output, double allowable_error){
			this(output,allowable_error,false);
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

		conformToState(State.ZERO);
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

	@Override
	public void readPeriodicInputs() {
		super.readPeriodicInputs();

		Logger.processInputs("EndEffectorWrist", mServoInputs);
	}

	@Override
	public void writePeriodicOutputs() {
		

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

		super.outputTelemetry();
	}

	@Override
	public void stop() {
	}

	@Override
	public boolean checkSystem() {
		return false;
	}
	public void conformToState(State state){
		setSetpointMotionMagic(state.output);
		mWantsHome = state.home;
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
				conformToState(_wantedState);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), _wantedState.output, _wantedState.allowable_error);
			}
		};
	}

}