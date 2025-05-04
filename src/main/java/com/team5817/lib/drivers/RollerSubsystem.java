package com.team5817.lib.drivers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.team5817.frc2025.loops.ILooper;

import lombok.Getter;

public class RollerSubsystem extends Subsystem {
    @Getter
    protected final List<RollerSubsystemBasic> rollers;
    private final Map<String, Integer> rollerNameToIndexMap;

    public RollerSubsystem(RollerSubsystemBasic.RollerSubsystemConstants... constantsArray) {
        rollers = new ArrayList<>();
        rollerNameToIndexMap = new HashMap<>();
        int index = 0;
        for (RollerSubsystemBasic.RollerSubsystemConstants constants : constantsArray) {
            rollers.add(new RollerSubsystemBasic(constants) {
            });
            rollerNameToIndexMap.put(constants.kName, index++);
        }
    }
    public RollerSubsystemBasic getRoller(int index) {
        return rollers.get(index);
    }
    public RollerSubsystemBasic getRoller(String name) {
        Integer index = rollerNameToIndexMap.get(name);
        if (index != null) {
            return rollers.get(index);
        }
        System.out.println("Roller with name " + name + " not found.");
        return null; // Return null if no roller with the given name is found
    }

    @Override
    public void outputTelemetry() {
        for (RollerSubsystemBasic roller: rollers) {
            roller.outputTelemetry();
        }
    }
    @Override
    public void readPeriodicInputs() {
        for (RollerSubsystemBasic roller: rollers) {
            roller.readPeriodicInputs();
        }
    }
    @Override
    public void writePeriodicOutputs() {
        for (RollerSubsystemBasic roller: rollers) {
            roller.writePeriodicOutputs();
        }
    }
    
    @Override
    public void stop() {
        for (RollerSubsystemBasic roller: rollers) {
            roller.stop();
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        for (RollerSubsystemBasic roller: rollers) {
            roller.registerEnabledLoops(enabledLooper);
        }
    }

    @Override
    public boolean checkSystem() {
        for (RollerSubsystemBasic roller: rollers) {
            if (!roller.checkSystem()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public boolean checkDeviceConfiguration() {
        for (RollerSubsystemBasic roller: rollers) {
            if (!roller.checkDeviceConfiguration()) {
                return false;
            }
        }
        return true;
    }
}
