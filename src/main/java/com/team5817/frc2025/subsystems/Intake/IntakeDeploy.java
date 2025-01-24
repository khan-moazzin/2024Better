package com.team5817.frc2025.subsystems.Intake;

import com.team5817.frc2025.Constants;
import com.team5817.frc2025.Robot;
import com.team5817.frc2025.Constants.IntakeDeployConstants;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.ServoMotorSubsystemWithCancoder;
import com.team5817.lib.requests.Request;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;


import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.lib.util.DelayedBoolean;
public class IntakeDeploy extends ServoMotorSubsystemWithCancoder {
	
	public static IntakeDeploy mInstance;

	public static IntakeDeploy getInstance() {
		if (mInstance == null) {
			mInstance = new IntakeDeploy(IntakeDeployConstants.kDeployServoConstants, IntakeDeployConstants.kDeployEncoderConstants);
		}
		return mInstance;
	}

	final static double kStrictError = .5;
	final static double kMediumError = 2;
	final static double kLenientError = 5;

    public enum State {
        DEPLOY(.3, kStrictError, true),//TODO
        CLEAR(0.0, kLenientError),//TODO
        UNJAM(0.0, kLenientError),//TODO
        STOW(0.0, kMediumError),
		ALGAE(0.0, kMediumError, true),
		HUMAN(0.0, kStrictError, true),
		ZERO(0.0, kStrictError);


        double output = 0;
		double allowable_error = 0;
		boolean needs_to_home = false;

        State(double output, double allowable_error, boolean needs_to_home) {
        this.output = output;
		this.allowable_error = allowable_error;
		this.needs_to_home = needs_to_home;
        }

        State(double output, double allowable_error) {
        this.output = output;
		this.needs_to_home = false;
		this.allowable_error = allowable_error;
        }
 
    }


	private boolean mHoming = false;
	private boolean mNeedsToHome = false;
	private final DelayedBoolean mHomingDelay =
			new DelayedBoolean(Timer.getTimestamp(), Constants.IntakeDeployConstants.kHomingTimeout);

	public IntakeDeploy(final ServoMotorSubsystemConstants constants, final AbsoluteEncoderConstants encoder_constants) {
		super(constants, encoder_constants);
		mMain.setPosition(homeAwareUnitsToRotations(0.0));
		enableSoftLimits(false);
		setSetpointMotionMagic(State.DEPLOY.output);
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				if (getSetpoint() == State.STOW.output && atHomingLocation() && mNeedsToHome && !mHoming) {
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
        Robot.mechPoses[0] = new Pose3d(new Translation3d(-.325,0,.261), new Rotation3d(Units.degreesToRadians(0),Units.rotationsToRadians(0.3-mServoInputs.position_units), Units.degreesToRadians(0)));
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

	public Request stateRequest(State _wantedState) {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(_wantedState.output);
				mNeedsToHome = _wantedState.needs_to_home;
			}
			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), _wantedState.output, _wantedState.allowable_error);
			}
		};
	}
}