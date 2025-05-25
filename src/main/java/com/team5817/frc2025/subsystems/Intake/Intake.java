package com.team5817.frc2025.subsystems.Intake;

import com.team5817.frc2025.loops.ILooper;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;

public class Intake extends Subsystem {

    private static final IntakeRollers mIntakeRollers = IntakeRollers.getInstance();
    private static final IndexerRollers mIndexerRollers = IndexerRollers.getInstance();

    private static Intake mInstance;

    /**
     * Gets the singleton instance of the Intake.
     *
     * @return The singleton instance.
     */
    public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    public enum State {
        IDLE(IntakeRollers.State.IDLE, IndexerRollers.State.OFF),
        INTAKING(IntakeRollers.State.INTAKING, IndexerRollers.State.RECEIVING),
        HALF_INTAKING(IntakeRollers.State.HALF_INTAKING, IndexerRollers.State.RECEIVING),
        EXHAUSTING(IntakeRollers.State.EXHAUSTING, IndexerRollers.State.OUTTAKING),
        IDLE_EXHAUST(IntakeRollers.State.IDLE_EXAUST, IndexerRollers.State.OFF);

        final IntakeRollers.State rollerState;
        final IndexerRollers.State indexerState;

        State(IntakeRollers.State rollerState, IndexerRollers.State indexerState) {
            this.rollerState = rollerState;
            this.indexerState = indexerState;
        }
    }

    private Intake() {
        // Private constructor
    }

    @Override
    public void readPeriodicInputs() {
        mIntakeRollers.readPeriodicInputs();
        mIndexerRollers.readPeriodicInputs();
    }

    @Override
    public void writePeriodicOutputs() {
        mIntakeRollers.writePeriodicOutputs();
        mIndexerRollers.writePeriodicOutputs();
    }

    @Override
    public void stop() {
        mIntakeRollers.stop();
        mIndexerRollers.stop();
    }

    @Override
    public boolean checkDeviceConfiguration() {
        return mIntakeRollers.checkDeviceConfiguration() && mIndexerRollers.checkDeviceConfiguration();
    }

    @Override
    public boolean checkSystem() {
        return mIntakeRollers.checkSystem() && mIndexerRollers.checkSystem();
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        mIntakeRollers.registerEnabledLoops(enabledLooper);
        mIndexerRollers.registerEnabledLoops(enabledLooper);
    }

    public Request stateRequest(State state) {
        return new ParallelRequest(
            mIntakeRollers.stateRequest(state.rollerState),
            mIndexerRollers.stateRequest(state.indexerState)
        );
    }
}
