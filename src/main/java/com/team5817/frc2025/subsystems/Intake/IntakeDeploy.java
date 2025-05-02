package com.team5817.frc2025.subsystems.Intake;

import com.team5817.frc2025.Robot;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.frc2025.subsystems.Intake.IntakeConstants.IntakeDeployConstants;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.ServoMotorSubsystemWithCancoder;
import com.team5817.lib.requests.Request;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

/**
 * The IntakeDeploy class controls the deployment mechanism of the intake system.
 */
public class IntakeDeploy extends ServoMotorSubsystemWithCancoder {

	public static IntakeDeploy mInstance;

	/**
	 * Gets the singleton instance of the IntakeDeploy class.
	 *
	 * @return The instance of the IntakeDeploy class.
	 */
	public static IntakeDeploy getInstance() {
		if (mInstance == null) {
			mInstance = new IntakeDeploy(IntakeDeployConstants.kDeployServoConstants, IntakeDeployConstants.kDeployEncoderConstants);
		}
		return mInstance;
	}

	final static double kStrictError = 20;
	final static double kMediumError = 50;
	final static double kLenientError = 80;
	private State mState = State.ZERO;
	/**
	 * Represents the different states of the intake deployment.
	 */
	public enum State {
		GROUND(-141, kStrictError), 
		CLEAR(0, kLenientError), 
		STOW(0, kMediumError),
		ALGAE(-78, kMediumError),
		HUMAN(-141, kStrictError),
		ZERO(0, kStrictError,true),
		DISABLE(true);

		double output = 0;
		double allowable_error = 0;
		boolean home = false;
		boolean disable = false;

		/**p
		 * Constructs a new State.
		 *
		 * @param output The output value for the state.
		 * @param allowable_error The allowable error for the state.
		 */
		State(double output, double allowable_error,boolean home) {
			this.output = output;
			this.allowable_error = allowable_error;
			this.home = home;
		}
		State(double output, double allowable_error){
			this(output, allowable_error,false);
		}
		State(boolean disable){
			this.disable = disable;
		}

	}


	/**
	 * Constructs a new IntakeDeploy subsystem.
	 *
	 * @param constants The constants for the servo motor subsystem.
	 * @param encoder_constants The constants for the absolute encoder.
	 */
	public IntakeDeploy(final ServoMotorSubsystemConstants constants, final AbsoluteEncoderConstants encoder_constants) {
		super(constants, encoder_constants);
		enableSoftLimits(false);
		setSetpointMotionMagic(State.GROUND.output);
	}


	/**
	 * Registers the enabled loops for the subsystem.
	 *
	 * @param enabledLooper The looper to register the loops with.
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

	/**
	 * Reads the periodic inputs for the subsystem.
	 */
	@Override
	public void readPeriodicInputs() {
		super.readPeriodicInputs();
		Logger.processInputs("IntakeDeploy", mServoInputs);
	}

	/**
	 * Writes the periodic outputs for the subsystem.
	 */
	@Override
	public void writePeriodicOutputs() {
		if(!mState.disable)
			setSetpointMotionMagic(mState.output);
		else
			setOpenLoop(0);
		super.writePeriodicOutputs();
	}

	/**
	 * Outputs telemetry data for the subsystem.
	 */
	@Override
	public void outputTelemetry() {
		Robot.mechPoses[0] = new Pose3d(new Translation3d(-.314, 0, .272), new Rotation3d(Units.degreesToRadians(0),
			Units.degreesToRadians(mServoInputs.position_units), Units.degreesToRadians(0)));

		Robot.desMechPoses[0] = new Pose3d(new Translation3d(-.314, 0, .272), new Rotation3d(Units.degreesToRadians(0),
				Units.degreesToRadians(demand), Units.degreesToRadians(0)));

		Logger.recordOutput(mConstants.kName+"/AtState",  stateRequest(mState).isFinished());
		Logger.recordOutput(mConstants.kName+"/State",	mState);

		super.outputTelemetry();
	}

	/**
	 * Stops the subsystem.
	 */
	@Override
	public void stop() {
	}

	/**
	 * Checks the system for any issues.
	 *
	 * @return True if the system is functioning correctly, false otherwise.
	 */
	@Override
	public boolean checkSystem() {
		return false;
	}
	public void conformToState(State state){
		mState = state;
	}

	/**
	 * Checks if the intake is at the homing location.
	 *
	 * @return True if the intake is within the homing zone, false otherwise.
	 */
	/**
	 * Creates a request to change the state of the intake deployment.
	 *
	 * @param _wantedState The desired state.
	 * @return The request to change the state.
	 */
	public Request stateRequest(State _wantedState) {
		return new Request() {
			@Override
			public void act() {
				if (mControlState != ControlState.MOTION_MAGIC) {
					mControlState = ControlState.MOTION_MAGIC;
				}
				conformToState(_wantedState);
			}

			@Override
			public boolean isFinished() {
				if(mState.disable)
					return true;
				return Util.epsilonEquals(getPosition(), _wantedState.output, _wantedState.allowable_error);
			}
		};
	}
}