package com.team5817.frc2025.subsystems.EndEffector;

import com.team5817.frc2025.Constants.EndEffectorWristConstants;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.frc2025.subsystems.Elevator.Elevator;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.ServoMotorSubsystemWithCancoder;
import com.team5817.lib.requests.Request;

import edu.wpi.first.wpilibj.util.Color8Bit;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import com.ctre.phoenix6.signals.NeutralModeValue;
public class EndEffectorWrist extends ServoMotorSubsystemWithCancoder {
	LoggedMechanismLigament2d two = Elevator.getInstance().midBar.append(new LoggedMechanismLigament2d("End Effector Wrist", 0.15, -90));
	LoggedMechanismLigament2d three = two.append(new LoggedMechanismLigament2d("End Effector Rod", 0.15, -90));

	
	public static EndEffectorWrist mInstance;

	public static EndEffectorWrist getInstance() {
		if (mInstance == null) {
			mInstance = new EndEffectorWrist(EndEffectorWristConstants.kWristServoConstants, EndEffectorWristConstants.kWristEncoderConstants);
		}
		return mInstance;
	}

	final static double kStrictError = .5/360;
	final static double kMediumError = 2/360;
	final static double kLenientError = 5/360;


    public enum State {

		L4(.4, kStrictError),
        L3(.4, kStrictError),
        L2(.4, kStrictError),
        L1(.4, kStrictError),
        A1(0.0, kMediumError),
        A2(0.0, kMediumError),
        NET(0.0, kMediumError),
        ZERO(.0, kLenientError),
        INTAKING(0.0, kStrictError),
        STOW(0.0, kStrictError);


        double output = 0;
		double allowable_error = 0;

        State(double output, double allowable_error) {
            this.output = output;
			this.allowable_error = allowable_error;
        }
    }



	public EndEffectorWrist(final ServoMotorSubsystemConstants constants, final AbsoluteEncoderConstants encoder_constants) {
		super(constants, encoder_constants);
		two.setLineWeight(3);
		three.setLineWeight(3);
		two.setColor(new Color8Bit(255,0,0));
		three.setColor(new Color8Bit(255,0,0));

		mMain.setPosition(homeAwareUnitsToRotations(120.0));
		enableSoftLimits(false);
		setSetpointMotionMagic(State.STOW.output);
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

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
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();
		
        Logger.processInputs("EndEffectorWrist", mServoInputs);
    }

	@Override
	public synchronized void writePeriodicOutputs() {
		super.writePeriodicOutputs();
	}

	@Override
	public void outputTelemetry() {
		two.setAngle(-90+8*mServoInputs.position_rots*360);
		super.outputTelemetry();
	}

	@Override
	public void stop() {}

	@Override
	public boolean checkSystem() {
		return false;
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