package com.team5817.frc2025.subsystems.Elevator;

import com.team5817.frc2025.Robot;
import com.team5817.frc2025.Constants.ElevatorConstants;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.ServoMotorSubsystem;
import com.team5817.lib.requests.Request;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

public class Elevator extends ServoMotorSubsystem {
	public static Elevator mInstance;

	public static Elevator getInstance() {
		if (mInstance == null) {
			mInstance = new Elevator(ElevatorConstants.kElevatorServoConstants);
		}
		return mInstance;
	}

	final static double kStrictError = .5;
	final static double kMediumError = 2;
	final static double kLenientError = 5;

	public enum State {
		L4(1.673, kStrictError),
		L3(.986, kStrictError),
		L2(0.563, kStrictError),
		L1(0.304, kStrictError),
		A1(0.59, kMediumError),
		A2(.896, kMediumError),
		NET(2, kMediumError),
		ZERO(0.0, kLenientError),
		PROCESS(0.0, kLenientError),
		STOW(0.0, kStrictError);

		double output = 0;
		double allowable_error = 20;

		State(double output, double allowable_error) {
			this.output = output;
			this.allowable_error = allowable_error;
		}
	}

	public Elevator(final ServoMotorSubsystemConstants constants) {
		super(constants);
		mMain.setPosition(homeAwareUnitsToRotations(0.0));
		enableSoftLimits(false);
		setSetpointMotionMagic(State.ZERO.output);
	}

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
				setNeutralMode(NeutralModeValue.Brake);
			}
		});
	}

	@Override
	public void readPeriodicInputs() {
		super.readPeriodicInputs();
		Logger.processInputs("Elevator", mServoInputs);
	}

	@Override
	public void writePeriodicOutputs() {
		super.writePeriodicOutputs();
	}

	@Override
	public void outputTelemetry() {
		Pose3d current = new Pose3d(Math.cos(Units.degreesToRadians(84)) * mServoInputs.position_units, 0,
				Math.sin(Units.degreesToRadians(84)) * mServoInputs.position_units, new Rotation3d());

		Robot.mechPoses[2] = current.div(3);
		Robot.mechPoses[3] = current.div(3).times(2);
		Robot.mechPoses[4] = current;

		Pose3d desired = new Pose3d(Math.cos(Units.degreesToRadians(84)) * mServoOutputs.demand, 0,
				Math.sin(Units.degreesToRadians(84)) * mServoOutputs.demand, new Rotation3d());

		Robot.desMechPoses[2] = desired.div(3);
		Robot.desMechPoses[3] = desired.div(3).times(2);
		Robot.desMechPoses[4] = desired;

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
	}

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

	public Request waitForExtensionRequest(double position) {
		return new Request() {
			@Override
			public void act() {
			}

			@Override
			public boolean isFinished() {
				return mServoInputs.position_units >= position;
			}
		};
	}

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