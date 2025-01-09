package com.team5817.frc2024.subsystems.Intake;

import com.team5817.lib.Util;
import com.team5817.lib.drivers.ServoMotorSubsystemWithCancoder;
import com.team5817.lib.requests.Request;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.lib.util.DelayedBoolean;
import com.team5817.frc2024.Constants;
import com.team5817.frc2024.Constants.IntakeDeployConstants;
import com.team5817.frc2024.loops.ILooper;
import com.team5817.frc2024.loops.Loop;
public class IntakeDeploy extends ServoMotorSubsystemWithCancoder {
	public static IntakeDeploy mInstance;

	public static IntakeDeploy getInstance() {
		if (mInstance == null) {
			mInstance = new IntakeDeploy(IntakeDeployConstants.kDeployServoConstants, IntakeDeployConstants.kDeployEncoderConstants);
		}
		return mInstance;
	}

    public enum State {
        DEPLOY(16.0),
        CLEAR(0.0),
        UNJAM(0.0),
        STOW(0.0);

        double output = 0;

        State(double output) {
        this.output = output;
        }
    }


	private boolean mHoming = false;
	private boolean mNeedsToHome = false;
	private final DelayedBoolean mHomingDelay =
			new DelayedBoolean(Timer.getTimestamp(), Constants.IntakeDeployConstants.kHomingTimeout);

	public IntakeDeploy(final ServoMotorSubsystemConstants constants, final AbsoluteEncoderConstants encoder_constants) {
		super(constants, encoder_constants);
		mMain.setPosition(homeAwareUnitsToRotations(120.0));
		enableSoftLimits(false);
		setSetpointMotionMagic(State.DEPLOY.output);
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				if (getSetpoint() == mConstants.kHomePosition && atHomingLocation() && mNeedsToHome && !mHoming) {
					setWantHome(true);
				} else if (mControlState != ControlState.OPEN_LOOP && mHoming) {
					setWantHome(false);
				}
			}

			@Override
			public void onStop(double timestamp) {
				setNeutralMode(NeutralModeValue.Brake);
			}
		});
	}

	/**
	 * Runs intake deploy rehoming sequence.
	 * @param home Enable/Disable rehome sequence.
	 */
	public void setWantHome(boolean home) {
		mHoming = home;

		if (mHoming) {
			mNeedsToHome = false;
		}
	}

    @Override
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();
        Logger.processInputs("IntakeDeploy", mServoInputs);
    }

	@Override
	public synchronized void writePeriodicOutputs() {
		if (mHoming) {
			setOpenLoop(Constants.IntakeDeployConstants.kHomingOutput / mConstants.kMaxForwardOutput);
			if (mHomingDelay.update(
					Timer.getFPGATimestamp(),
					Math.abs(getVelocity()) < Constants.IntakeDeployConstants.kHomingVelocityWindow)) {
				zeroSensors();
				mHasBeenZeroed = true;
				setSetpointMotionMagic(mConstants.kHomePosition);
				mHoming = false;
			}
		}
		super.writePeriodicOutputs();
	}

	@Override
	public void outputTelemetry() {
        Logger.recordOutput("IntakeDeploy/Homing", mHoming);
        Logger.recordOutput("IntakeDeploy/Within Homing Window", atHomingLocation());

		SmartDashboard.putBoolean(mConstants.kName + "/Homing", mHoming);
		SmartDashboard.putBoolean(mConstants.kName + "/Within Homing Window", atHomingLocation());
		super.outputTelemetry();
	}

	@Override
	public void stop() {}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public boolean atHomingLocation() {
		// Check if intake is  within 7.0 degrees of homing position
		return mServoInputs.position_units - mConstants.kHomePosition > -Constants.IntakeDeployConstants.kHomingZone;
	}

	/**
	 * @return Request to move intake to deploy angle to pick up notes off the ground.
	 */
	public Request deployRequest() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.DEPLOY.output);
				mNeedsToHome = true;
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.DEPLOY.output, 20.0);
			}
		};
	}

	/**
	 * @return Request to move intake to stow angle.
	 */
	public Request tuckRequest() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.STOW.output);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.STOW.output, 4.0);
			}
		};
	}

	/**
	 * @return Request to move intake in order to avoid collisions with the elevator.
	 */
	public Request clearRequest() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.CLEAR.output);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.CLEAR.output, 4.0);
			}
		};
	}

	/**
	 * @return Request to move intake in order to exhaust notes forwards, out of the robot.
	 */
	public Request unjamRequest() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.UNJAM.output);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.UNJAM.output, 4.0);
			}
		};
	}
}