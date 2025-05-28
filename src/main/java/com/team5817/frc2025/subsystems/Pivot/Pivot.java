package com.team5817.frc2025.subsystems.Pivot;

import com.team5817.frc2025.RobotVisualizer;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.State.ServoState;
import com.team5817.lib.drivers.StateBasedServoMotorSubsystem;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lombok.Getter;

import org.littletonrobotics.junction.Logger;

/**
 * Pivot subsystem for controlling the pivot mechanism.
 */
public class Pivot extends StateBasedServoMotorSubsystem<Pivot.State> {
    public static Pivot mInstance;

    /**
     * Returns the singleton instance of the Pivot.
     * 
     * @return the singleton instance of the Pivot
     */
    public static Pivot getInstance() {
        if (mInstance == null) {
            mInstance = new Pivot(PivotConstants.kPivotServoConstants);
        }
        return mInstance;
    }

    private double distanceFromScoringPosition = 0;
    private double scoringOffset = 0;

    public enum State implements ServoState {
        MAXUP(PivotConstants.State.MAX_UP),
        MAXDOWN(PivotConstants.State.MAX_DOWN),
        INTAKING(PivotConstants.State.INTAKING),
        TRANSFER(PivotConstants.State.TRANSFER),
        CLEAR(0.35), // if no constant, keep the literal
        STOW(PivotConstants.State.MAX_DOWN); // assuming stow = MAX_DOWN

        @Getter private double demand = 0;
        private InterpolatingDoubleTreeMap map;

        State(double output) {
            this(output, null);
        }

        State(double output, InterpolatingDoubleTreeMap map) {
            this.demand = output;
            this.map = map;
        }

        public double getTrackedOutput(double distanceFromScoringPosition) {
            if (map == null) {
                return demand;
            }
            double des = this.demand + map.get(distanceFromScoringPosition);
            des = Util.limit(des, PivotConstants.kPivotServoConstants.kMinUnitsLimit, PivotConstants.kPivotServoConstants.kMaxUnitsLimit);
            return des;
        }

        public boolean isDisabled() {
            return false;
        }

        @Override
        public ControlState getControlState() {
            return ControlState.MOTION_MAGIC;
        }

        @Override
        public double getAllowableError() {
            return 0.1;
        }
    }

    /**
     * Constructs a Pivot with the given constants.
     * 
     * @param constants the constants for the pivot
     */
    public Pivot(final ServoMotorSubsystemConstants constants) {
        super(constants, State.STOW, false);
        enableSoftLimits(false);
    }

    public void setManualOffset(double offset) {
        this.scoringOffset = offset;
    }

    public void changeManualOffset(double deltaOffset) {
        this.scoringOffset += deltaOffset;
    }

    @Override
    public void writePeriodicOutputs() {
        double trackedOutput = mState.getTrackedOutput(distanceFromScoringPosition);
        if (mState == State.MAXUP || mState == State.MAXDOWN) {//if aiming, use map
            trackedOutput += scoringOffset;
        }

        if (mControlState == ControlState.MOTION_MAGIC)
            setSetpointMotionMagic(trackedOutput);
        super.writePeriodicOutputs();
    }

    @Override
    public void outputTelemetry() {
        RobotVisualizer.updatePivotHeight(getPosition());

        Logger.recordOutput("Pivot/Offset", this.scoringOffset);

        super.outputTelemetry();
    }
}
