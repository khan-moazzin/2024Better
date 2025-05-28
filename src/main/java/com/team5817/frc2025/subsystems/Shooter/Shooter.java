package com.team5817.frc2025.subsystems.Shooter;

import com.team5817.frc2025.subsystems.Shooter.ShooterConstants.ShooterMotorConstants;
import com.team5817.lib.drivers.BasicStateBasedRollerSubsystem;
import com.team5817.lib.drivers.State.BasicRollerState;

import lombok.Getter;

/**
 * The EndEffectorRollers subsystem controls the rollers of the end effector.
 */
public class Shooter extends BasicStateBasedRollerSubsystem<Shooter.State> {
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
	
	public enum State implements BasicRollerState {
		PARTIALRAMP(.7),
    	SHOOTING(1),
    	TRANSFER(0.3),
    	REVERSETRANSFER(-.5),
    	AMP(.5),
    	IDLE(.1);

		@Getter private double rollerDemand;

		double output = 0;
    	State(double output){
        	this.output = output;
    }

		@Override
		public ControlState getControlState() {
			return ControlState.DUTY_CYCLE;
		}
	}




	
	/**
	 * Private constructor for the EndEffectorRollers subsystem.
	 */
	private Shooter() {
		super(State.IDLE, ShooterConstants.ShooterMotorConstants.kShooterConstants);
	}
}