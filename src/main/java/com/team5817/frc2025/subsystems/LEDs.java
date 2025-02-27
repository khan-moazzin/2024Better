package com.team5817.frc2025.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdleControlFrame;
import com.ctre.phoenix.led.CANdleStatusFrame;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.frc2025.Constants;
import com.team5817.frc2025.Ports;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.frc2025.subsystems.vision.VisionDeviceManager;
import com.team5817.lib.Lights.Color;
import com.team5817.lib.Lights.TimedLEDState;
import com.team5817.lib.requests.Request;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class LEDs extends Subsystem {
	private static LEDs mInstance;

	/**
	 * Gets the singleton instance of the LEDs subsystem.
	 *
	 * @return The singleton instance of LEDs.
	 */
	public static LEDs getInstance() {
		if (mInstance == null) {
			mInstance = new LEDs();
		}
		return mInstance;
	}

	private final int kNumLeds = 8 + 18;

	private final CANdle mCandle;
	private LEDSection mLEDStatus = new LEDSection(0, kNumLeds);

	/**
	 * Constructor for the LEDs subsystem.
	 */
	public LEDs() {
		mCandle = new CANdle(Ports.LEDS.getDeviceNumber(), Ports.LEDS.getBus());
		CANdleConfiguration configAll = new CANdleConfiguration();
		configAll.statusLedOffWhenActive = false;
		configAll.disableWhenLOS = true;
		configAll.stripType = LEDStripType.RGB;
		configAll.brightnessScalar = 1.0;
		configAll.vBatOutputMode = VBatOutputMode.Modulated;
		mCandle.configAllSettings(configAll, (int) (1000 * Constants.kLongCANTimeoutS));
		mCandle.setStatusFramePeriod(CANdleStatusFrame.CANdleStatusFrame_Status_1_General, 255);
		mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_1_General, 10);
		mCandle.setControlFramePeriod(CANdleControlFrame.CANdle_Control_2_ModulatedVBatOut, 255);
		applyStates(TimedLEDState.DISABLE_BLUE);
	}

	/**
	 * Registers the enabled loops for the LEDs subsystem.
	 *
	 * @param mEnabledLooper The enabled looper to register.
	 */
	@Override
	public void registerEnabledLoops(ILooper mEnabledLooper) {
		mEnabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				applyStates(TimedLEDState.OFF);
			}

			@Override
			public void onLoop(double timestamp) {
			}
		});
	}

	/**
	 * Reads the periodic inputs for the LEDs subsystem.
	 */
	@Override
	public void readPeriodicInputs() {
		if (DriverStation.isDisabled()) {
			if (!VisionDeviceManager.getInstance().fullyConnected()) {
				applyStates(TimedLEDState.NO_VISION);
			} else {
				if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
					applyStates(TimedLEDState.DISABLE_RED);
				} else {
					applyStates(TimedLEDState.DISABLE_BLUE);
				}
			}
		}

		double timestamp = Timer.getTimestamp();
		if (mLEDStatus.state.interval != Double.POSITIVE_INFINITY) {
			if (timestamp - mLEDStatus.lastSwitchTime >= mLEDStatus.state.interval) {
				mLEDStatus.nextColor();
				mLEDStatus.lastSwitchTime = timestamp;
			}
		}

		Color color = mLEDStatus.getWantedColor();
		mCandle.setLEDs(color.r, color.g, color.b, 0, mLEDStatus.startIDx, 100);
	}

	/**
	 * Applies the given TimedLEDState to the LEDs.
	 *
	 * @param TimedState The TimedLEDState to apply.
	 */
	public void applyStates(TimedLEDState TimedState) {
		mLEDStatus.setState(TimedState);
	}

	/**
	 * Gets the current TimedLEDState of the LEDs.
	 *
	 * @return The current TimedLEDState.
	 */
	public TimedLEDState getState() {
		return mLEDStatus.state;
	}

	@Override
	public void stop() {
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void outputTelemetry() {
	}

	/**
	 * Creates a request to update the LEDs to a specific color or animation.
	 *
	 * @param wanted_state The desired LED color/animation.
	 * @return A Request to update the LEDs.
	 */
	public Request stateRequest(TimedLEDState wanted_state) {
		return new Request() {

			@Override
			public void act() {
				applyStates(wanted_state);
			}

			@Override
			public boolean isFinished() {
				return true;
			}
		};
	}

	// Class for holding information about each section
	private class LEDSection {
		private TimedLEDState state = TimedLEDState.OFF; // current TimedState
		private double lastSwitchTime = 0.0; // timestampe of last color cycle
		private int colorIndex = 0; // tracks current color in array
		private int startIDx, LEDCount; // start and end of section

		/**
		 * Constructor for LEDSection.
		 *
		 * @param startIndex The start index of the LED section.
		 * @param endIndex The end index of the LED section.
		 */
		public LEDSection(int startIndex, int endIndex) {
			startIDx = startIndex;
			LEDCount = endIndex - startIndex;
		}

		/**
		 * Sets the state of the LED section.
		 *
		 * @param wantedTimedState The desired TimedLEDState.
		 */
		public void setState(TimedLEDState wantedTimedState) {
			if (wantedTimedState != state) {
				colorIndex = 0;
				lastSwitchTime = Timer.getTimestamp();
				state = wantedTimedState;
			}
		}

		/**
		 * Gets the current color of the LED section.
		 *
		 * @return The current color.
		 */
		public Color getWantedColor() {
			Color color;
			try {
				color = state.colors[colorIndex];
			} catch (Exception e) {
				color = Color.off();
			}
			return color;
		}

		/**
		 * Cycles to the next color in the array.
		 */
		public void nextColor() {
			if (state.colors.length == 1) {
				return;
			}
			if (colorIndex == state.colors.length - 1) {
				colorIndex = 0;
			} else {
				colorIndex++;
			}
		}

		/**
		 * Resets the LED section to the default state.
		 */
		public void reset() {
			state = TimedLEDState.OFF;
			lastSwitchTime = 0.0;
			colorIndex = 0;
		}
	}
}
