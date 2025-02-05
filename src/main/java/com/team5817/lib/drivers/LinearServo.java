package com.team5817.lib.drivers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class representing a linear servo, which extends the Servo class.
 * This class provides methods to control and monitor the position of a linear servo.
 */
public class LinearServo extends Servo {
	double m_speed;
	double m_length;
	double setPos;
	double curPos;

	/**
	 * Constructs a LinearServo with the specified PWM channel, length, and speed.
	 *
	 * @param channel the PWM channel used to control the servo
	 * @param length  the maximum length of the servo in millimeters
	 * @param speed   the maximum speed of the servo in millimeters per second
	 */
	public LinearServo(int channel, int length, int speed) {
		super(channel);
		setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
		m_length = length;
		m_speed = speed;
	}

	/**
	 * Sets the target position of the servo.
	 * This method should be called periodically to update the servo's position.
	 *
	 * @param setpoint the target position of the servo in millimeters
	 */
	public void setPosition(double setpoint) {
		setPos = MathUtil.clamp(setpoint, 0, m_length);
		setSpeed((setPos / m_length * 2) - 1);
		SmartDashboard.putNumber("Linear Servo Setpoint " + this.getChannel(), (setPos / m_length * 2) - 1);
	}

	double lastTime = 0;

	/**
	 * Updates the current position estimation of the servo.
	 * This method should be called periodically to ensure accurate position tracking.
	 */
	public void updateCurPos() {
		double dt = Timer.getTimestamp() - lastTime;
		if (curPos > setPos + m_speed * dt) {
			curPos -= m_speed * dt;
		} else if (curPos < setPos - m_speed * dt) {
			curPos += m_speed * dt;
		} else {
			curPos = setPos;
		}
	}

	/**
	 * Returns the current position of the servo.
	 * The {@link #updateCurPos() updateCurPos()} method must be called periodically for accurate results.
	 *
	 * @return the current position of the servo in millimeters
	 */
	public double getPosition() {
		return curPos;
	}

	/**
	 * Checks if the servo has reached its target position.
	 * The {@link #updateCurPos() updateCurPos()} method must be called periodically for accurate results.
	 *
	 * @return true if the servo is at its target position, false otherwise
	 */
	public boolean isFinished() {
		return curPos == setPos;
	}
}
