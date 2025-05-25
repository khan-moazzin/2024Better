package com.team5817.frc2025.subsystems.Intake;

import com.team5817.frc2025.subsystems.Intake.IntakeConstants.RollerConstants;
import com.team5817.lib.drivers.RollerSubsystemBasic.ControlState;
import com.team5817.lib.drivers.State.RollerState;
import com.team5817.lib.drivers.StateBasedRollerSubsystem;
import lombok.Getter;


public class IndexerRollers extends StateBasedRollerSubsystem<IndexerRollers.State> {

    private static IndexerRollers mInstance;

    /**
     * Gets the singleton instance of the Indexer class.
     *
     * @return The singleton instance.
     */
    public static IndexerRollers getInstance() {
        if (mInstance == null) {
            mInstance = new IndexerRollers();
        }
        return mInstance;
    }

    public enum State implements RollerState {
        OFF(0, ControlState.VOLTAGE),
        RECEIVING(-0.2, ControlState.VOLTAGE),
        TRANSFERRING(-1.0, ControlState.VOLTAGE),
        OUTTAKING(0.4, ControlState.VOLTAGE);

        @Getter private double[] rollerDemands;
        @Getter private ControlState[] controlStates;
        @Getter private boolean disabled = false;

        State(double demand, ControlState control) {
            this.rollerDemands = new double[] { demand };
            this.controlStates = new ControlState[] { control };
        }

    }

    private IndexerRollers() {
        super(State.OFF, RollerConstants.kIndexerConstants);
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
    }
}
