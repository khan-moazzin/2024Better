package com.team5817.lib.drivers;

import static edu.wpi.first.units.Units.Rotation;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.team254.lib.drivers.CanDeviceId;

/**
 * Abstract class representing a servo motor subsystem with a CANcoder for absolute position feedback.
 */
public abstract class ServoMotorSubsystemWithCancoder extends ServoMotorSubsystem {

	/**
	 * Constants for configuring the absolute encoder.
	 */
	public static class AbsoluteEncoderConstants {
		public CanDeviceId remote_encoder_port;
		public double rotor_to_sensor_ratio;
	}

	private CANcoder mCancoder;
	private AbsoluteEncoderConstants mEncoderConstants;

	/**
	 * Constructs a ServoMotorSubsystemWithCancoder.
	 *
	 * @param constants         The constants for the servo motor subsystem.
	 * @param encoder_constants The constants for the absolute encoder.
	 */
	protected ServoMotorSubsystemWithCancoder(
			ServoMotorSubsystemConstants constants, AbsoluteEncoderConstants encoder_constants) {
		super(constants);
		mEncoderConstants = encoder_constants;

		mCancoder = new CANcoder(
				encoder_constants.remote_encoder_port.getDeviceNumber(),
				encoder_constants.remote_encoder_port.getBus());

		Logger.recordOutput("absolute encoder reset", mCancoder.getAbsolutePosition().getValue().in(Rotation) * mEncoderConstants.rotor_to_sensor_ratio);
		mMain.setPosition((mCancoder.getAbsolutePosition().getValue().in(Rotation))*mEncoderConstants.rotor_to_sensor_ratio);
	}

	/**
	 * Zeros the sensors. This implementation does nothing.
	 */
	@Override
	public void zeroSensors() {
		return;
	}

	/**
	 * Sets the position of the CANcoder.
	 *
	 * @param value The position value to set.
	 */
	public void setPosition(double value) {
		mCancoder.setPosition(value);
	}

	/**
	 * Converts rotations to homed rotations, taking into account the encoder offset.
	 *
	 * @param input The input rotations.
	 * @return The homed rotations.
	 */
	@SuppressWarnings("unused")
	private double rotationsToHomedRotations(double input) {
		double rot = 0;
		while (rot > 1.0) {
			rot--;
		}
		while (rot < 0.0) {
			rot++;
		}
		return rot;
	}
}
