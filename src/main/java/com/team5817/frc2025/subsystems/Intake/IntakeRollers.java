package com.team5817.frc2025.subsystems.Intake;

import com.team5817.lib.drivers.RollerSubsystemBasic.ControlState;
import com.team5817.lib.drivers.State.RollerState;
import com.team5817.lib.drivers.StateBasedRollerSubsystem;
import com.team5817.frc2025.subsystems.Intake.IntakeConstants.RollerConstants;
import lombok.Getter;

/**
 * The IntakeRollers class controls the intake roller subsystem using 2024 intake logic.
 */
public class IntakeRollers extends StateBasedRollerSubsystem<IntakeRollers.State> {

    private static IntakeRollers mInstance;

    /**
     * Gets the singleton instance of the IntakeRollers.
     *
     * @return The singleton instance.
     */
    public static IntakeRollers getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeRollers();
        }
        return mInstance;
    }

    /**
     * Represents the different states of the intake rollers.
     */
    public enum State implements RollerState {
        IDLE(0, ControlState.VOLTAGE),
        INTAKING(0.75, ControlState.VOLTAGE),
        HALF_INTAKING(0.75, ControlState.VOLTAGE),
        EXHAUSTING(-0.7, ControlState.VOLTAGE),
        IDLE_EXAUST(0, ControlState.VOLTAGE);

        @Getter private final double[] rollerDemands;
        @Getter private final ControlState[] controlStates;

        State(double demand, ControlState control) {
            this.rollerDemands = new double[] { demand };
            this.controlStates = new ControlState[] { control };
        }
    }

    /**
     * Private constructor for the IntakeRollers subsystem.
     */
    private IntakeRollers() {
        super(State.IDLE, RollerConstants.kIntakeConstants);
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
    }
}
