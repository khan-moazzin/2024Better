package com.team5817.frc2025.subsystems.Intake;

import com.team5817.frc2025.Robot;
import com.team5817.frc2025.subsystems.Intake.IntakeConstants.DeployConstants;
import com.team5817.lib.drivers.State.ServoState;
import com.team5817.lib.drivers.StateBasedServoMotorSubsystemWithCancoder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import lombok.Getter;

/**
 * The IntakeDeploy class controls the deployment mechanism of the intake system.
 */
public class IntakeDeploy extends StateBasedServoMotorSubsystemWithCancoder<IntakeDeploy.State> {

	public static IntakeDeploy mInstance;

	/**
	 * Gets the singleton instance of the IntakeDeploy class.
	 *
	 * @return The instance of the IntakeDeploy class.
	 */
	public static IntakeDeploy getInstance() {
		if (mInstance == null) {
			mInstance = new IntakeDeploy(DeployConstants.kDeployServoConstants, DeployConstants.kDeployEncoderConstants);
		}
		return mInstance;
	}

	final static double kStrictError = 20;
	final static double kMediumError = 50;
	final static double kLenientError = 80;

	/**
	 * Represents the different states of the intake deployment.
	 */
	public enum State implements ServoState {
		GROUND(-141, kStrictError), 
		STOW(0, kMediumError),
		HUMAN(-141, kStrictError),
		ZERO(0, kStrictError),
		DISABLE();

		@Getter private double demand = 0;
		@Getter private double allowableError = 0;
		@Getter private boolean disabled = false;

		/**p
		 * Constructs a new State.
		 *
		 * @param output The output value for the state.
		 * @param allowable_error The allowable error for the state.
		 */
		State(double output, double allowable_error) {
			this.demand = output;
			this.allowableError = allowable_error;
		}
		
		State(){this.disabled = true;}

		@Override
		public ControlState getControlState() {
			return ControlState.MOTION_MAGIC;
		}
	}

	/**
	 * Constructs a new IntakeDeploy subsystem.
	 *
	 * @param constants The constants for the servo motor subsystem.
	 * @param encoder_constants The constants for the absolute encoder.
	 */
	public IntakeDeploy(final ServoMotorSubsystemConstants constants, final AbsoluteEncoderConstants encoder_constants) {
		super(constants, encoder_constants, IntakeDeploy.State.GROUND);
		enableSoftLimits(false);
	}

	/**
	 * Outputs telemetry data for the subsystem.
	 */
	@Override
	public void outputTelemetry() {
		Robot.mechPoses[0] = new Pose3d(new Translation3d(-.314, 0, .272), new Rotation3d(Units.degreesToRadians(0),
			Units.degreesToRadians(mServoInputs.position_units), Units.degreesToRadians(0)));

		super.outputTelemetry();
	}
}