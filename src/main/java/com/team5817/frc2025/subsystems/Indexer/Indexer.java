package com.team5817.frc2025.subsystems.Indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonUtil;
import com.team5817.frc2025.Ports;
import com.team5817.frc2025.Constants.IntakeRollerConstants;
import com.team5817.frc2025.loops.ILooper;
import com.team5817.frc2025.loops.Loop;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.Request;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends Subsystem {
	private static Indexer mInstance;

	/**
	 * Gets the singleton instance of the Indexer.
	 *
	 * @return The singleton instance.
	 */
	public static Indexer getInstance() {
		if (mInstance == null) {
			mInstance = new Indexer();
		}
		return mInstance;
	}

	public enum State {
		IDLE(0.0,0),
		INDEXING(1,8.0),
		EXHAUST(-6.0,-6.0),
		IDLE_EXAUST(-2,-2);

		public double side_voltage;
		public double bottom_voltage;

		State(double bottom_voltage,double roller_voltage) {
			this.side_voltage = roller_voltage;
			this.bottom_voltage  =bottom_voltage;
		}
	}

	private final TalonFX SideRollers;
	private final TalonFX BottomRollers;

	private IndexerInputsAutoLogged mIndexerInputs = new IndexerInputsAutoLogged();

	/**
	 * Private constructor for the Indexer subsystem.
	 */
	private Indexer() {
		SideRollers = new TalonFX(Ports.SIDE_INDEXER.getDeviceNumber(), Ports.SIDE_INDEXER.getBus());
		BottomRollers = new TalonFX(Ports.BOTTOM_INDEXER.getDeviceNumber(), Ports.BOTTOM_INDEXER.getBus());
		TalonUtil.applyAndCheckConfiguration(SideRollers, IntakeRollerConstants.RollerFXConfig());
		TalonUtil.applyAndCheckConfiguration(BottomRollers, IntakeRollerConstants.RollerFXConfig());
	}

	/**
	 * Registers the enabled loops for the Indexer.
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

	@AutoLog
	public static class IndexerInputs implements Sendable {
		// Inputs
		public double roller_output_voltage;
		public double roller_stator_current;
		public double roller_velocity;
		public double roller_temperature;

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("VelocityRpS", () -> roller_velocity, null);
			builder.addDoubleProperty("OutputVoltage", () -> roller_output_voltage, null);
			builder.addDoubleProperty("StatorCurrent", () -> roller_stator_current, null);
			builder.addDoubleProperty("RoolerTemp", () -> roller_temperature, null);
		}
	}

		// OUTPUTS
		public State mState = State.IDLE;

	

	/**
	 * Gets the current state of the intake rollers.
	 *
	 * @return The current state.
	 */
	public State getState() {
		return mState;
	}

	/**
	 * Sets the state of the intake rollers.
	 *
	 * @param state The state to set.
	 */
	public void setState(State state) {
		mState = state;
	}

	/**
	 * Creates a new request that updates the intake rollers with the wanted state.
	 *
	 * @param _wantedState Wanted state for the intake rollers.
	 * @return New request that updates the intake rollers with the wanted state.
	 */
	public Request stateRequest(State _wantedState) {
		return new Request() {
			@Override
			public void act() {
				setState(_wantedState);
			}

			@Override
			public boolean isFinished() {
				return true;
			}
		};
	}

	@Override
	public void readPeriodicInputs() {
		mIndexerInputs.roller_output_voltage = SideRollers.getMotorVoltage().getValue().in(Volts);
		mIndexerInputs.roller_stator_current = SideRollers.getStatorCurrent().getValue().in(Amps);
		mIndexerInputs.roller_velocity = SideRollers.getVelocity().getValue().in(RotationsPerSecond);
		mIndexerInputs.roller_temperature = SideRollers.getDeviceTemp().getValue().in(Fahrenheit);

		Logger.processInputs("Indexer", mIndexerInputs);
	}

	@Override
	public void writePeriodicOutputs() {
		SideRollers.setControl(new VoltageOut(mState.side_voltage));
		BottomRollers.setControl(new VoltageOut(mState.bottom_voltage));
	}

	@Override
	public void stop() {
		mState = State.IDLE;
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void outputTelemetry() {
	}

}