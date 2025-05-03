package com.team5817.lib.drivers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import lombok.Getter;

public class RollerSubsystem extends Subsystem {
    @Getter private final List<RollerSubsystemBasic> rollers;
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
}
