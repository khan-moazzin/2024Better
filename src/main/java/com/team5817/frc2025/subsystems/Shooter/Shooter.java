package com.team5817.frc2025.subsystems.Shooter;

import com.team5817.frc2025.subsystems.Shooter.ShooterConstants.ShooterMotorConstants;
import com.team5817.lib.drivers.StateBasedRollerSubsystem;
import com.team5817.lib.drivers.RollerSubsystemBasic.ControlState;
import com.team5817.lib.drivers.State.RollerState;
import com.team5817.lib.drivers.StateBasedRollerSubsystem;

import lombok.Getter;

/**
 * The EndEffectorRollers subsystem controls the rollers of the end effector.
 */
public class Shooter extends StateBasedRollerSubsystem<Shooter.State> {
	private static Shooter mInstance;

	/**
	 * Gets the singleton instance of the EndEffectorRollers subsystem.
	 *
	 * @return The singleton instance.
	 */
	public static Shooter getInstance() {
		if (mInstance == null) {
			mInstance = new Shooter();
		}
		return mInstance;
	}
	
	public enum State implements RollerState {
		PARTIALRAMP(0.7),
		SHOOTING(1.0),
		TRANSFER(0.3),
		REVERSETRANSFER(-0.5),
		AMP(0.5),
		IDLE(0.1);

		@Getter public final double rollerDemand;

			State(double output) {
				this.rollerDemand = output;
			}

		@Override
		public double[] getRollerDemands() {
			return new double[] { rollerDemand, rollerDemand }; 
		}

		@Override
		public ControlState[] getControlStates() {
			return new ControlState[] { ControlState.DUTY_CYCLE, ControlState.DUTY_CYCLE };
		}

		
}
	private double spinMultiplier = 1;
	public void setSpin(double spinMultiplier){
		this.spinMultiplier = spinMultiplier;
	}


	private double velocitySetpoint = 0.0;

	public void setVelocitySetpoint(double velocity) {
		this.velocitySetpoint = velocity;
	}

	public double getVelocitySetpoint() {
	return velocitySetpoint;
	}
	
	public void conformToState(State state) {
			setState(state); 
		}


	/**
	 * Private constructor for the Shooter subsystem.
	 */
	private Shooter() {
		super(State.IDLE, ShooterConstants.ShooterMotorConstants.kShooterConstants);
	}
}