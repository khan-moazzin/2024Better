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

	final static double kStrictError = .5 / 360;
	final static double kMediumError = 2 / 360;
	final static double kLenientError = 5 / 360;

	public enum State {

		L4(82.2, kStrictError),
		L3(0, kStrictError),
		L2(15, kStrictError),
		L1(0, kStrictError),
		A1(208, kMediumError),
		A2(208, kMediumError),
		NET(208, kMediumError),
		ZERO(0, kLenientError),
		INTAKING(150, kStrictError),
		STOW(150, kStrictError);

		double output = 0;
		double allowable_error = 0;

		State(double output, double allowable_error) {
			this.output = output;
			this.allowable_error = allowable_error;
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

		mMain.setPosition(homeAwareUnitsToRotations(0));
		enableSoftLimits(false);
		setSetpointMotionMagic(State.STOW.output);
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
						Units.rotationsToRadians(-.48 + mServoInputs.position_units), Units.degreesToRadians(0))));

		Robot.desMechPoses[5] = Robot.desMechPoses[4]
				.transformBy(new Transform3d(new Translation3d(.221, 0, .278), new Rotation3d(Units.degreesToRadians(0),
						Units.rotationsToRadians(-.48 + mServoOutputs.demand), Units.degreesToRadians(0))));

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
	 * Returns a request to stow the end effector wrist.
	 * 
	 * @return The stow request.
	 */
	public Request stowRequest() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.STOW.output);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.STOW.output, State.STOW.allowable_error);
			}
		};
	}

	/**
	 * Returns a request to zero the end effector wrist.
	 * 
	 * @return The zero request.
	 */
	public Request zeroRequest() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.ZERO.output);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.ZERO.output, State.ZERO.allowable_error);
			}
		};
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
				setSetpointMotionMagic(_wantedState.output);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), _wantedState.output, _wantedState.allowable_error);
			}
		};
	}

}