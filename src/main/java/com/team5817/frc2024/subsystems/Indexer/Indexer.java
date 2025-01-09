package com.team5817.frc2024.subsystems.Indexer;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team254.lib.drivers.TalonUtil;
import com.team5817.frc2024.Constants.IntakeRollerConstants;
import com.team5817.frc2024.Ports;
import com.team5817.frc2024.loops.ILooper;
import com.team5817.frc2024.loops.Loop;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.Request;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexer extends Subsystem {
	private static Indexer mInstance;

	public static Indexer getInstance() {
		if (mInstance == null) {
			mInstance = new Indexer();
		}
		return mInstance;
	}

	public enum State {
		IDLE(0.0),
		INDEXING(8.0),
		EXHAUST(-6.0);

		public double roller_voltage;

		State(double roller_voltage) {
			this.roller_voltage = roller_voltage;
		}
	}

	private final TalonFX mRoller;

	private State mState = State.IDLE;
	private IndexerInputsAutoLogged mIndexerInputs = new IndexerInputsAutoLogged();
	private IndexerOutputsAutoLogged mIndexerOutputs = new IndexerOutputsAutoLogged();

	private Indexer() {
		mRoller = new TalonFX(Ports.INTAKE_ROLLER.getDeviceNumber(), Ports.INTAKE_ROLLER.getBus());
		TalonUtil.applyAndCheckConfiguration(mRoller, IntakeRollerConstants.RollerFXConfig());
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				mIndexerOutputs.roller_demand = mState.roller_voltage;
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}
	
	@AutoLog
	public static class IndexerInputs implements Sendable {
		// Inputs
		public double roller_output_voltage;
		public double roller_stator_current;
		public double roller_velocity;


		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("VelocityRpS", () -> roller_velocity, null);
			builder.addDoubleProperty("OutputVoltage", () -> roller_output_voltage, null);
			builder.addDoubleProperty("StatorCurrent", () -> roller_stator_current, null);
		}
	}

	@AutoLog
	public static class IndexerOutputs implements Sendable {
		// OUTPUTS
		public double roller_demand;

		@Override
		public void initSendable(SendableBuilder builder) {
			builder.addDoubleProperty("Demand", () -> roller_demand, null);
		}
	}

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
				return mIndexerOutputs.roller_demand == _wantedState.roller_voltage;
			}
		};
	}

	@Override
	public void readPeriodicInputs() {
		mIndexerInputs.roller_output_voltage = mRoller.getMotorVoltage().getValue().in(Volts);
		mIndexerInputs.roller_stator_current = mRoller.getStatorCurrent().getValue().in(Amps);
		mIndexerInputs.roller_velocity = mRoller.getVelocity().getValue().in(RotationsPerSecond);

		Logger.processInputs("Indexer", mIndexerInputs);
	}

	@Override
	public void writePeriodicOutputs() {
		mRoller.setControl(new VoltageOut(mIndexerOutputs.roller_demand));
	}

	@Override
	public void stop() {
		mIndexerOutputs.roller_demand = 0.0;
	}

	@Override
	public boolean checkSystem() {
		return false;
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putString("IntakeRollers/State", mState.toString());
		SmartDashboard.putData("IntakeRollers/I", mIndexerInputs);
		SmartDashboard.putData("IntakeRollers/O", mIndexerOutputs);
	}
}