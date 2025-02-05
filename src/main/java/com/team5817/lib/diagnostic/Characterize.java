package com.team5817.lib.diagnostic;

import com.team5817.frc2025.autos.AutoBase;
import com.team5817.lib.diagnostic.FeedForwardCharacterization.FeedForwardCharacterizationData;
import com.team5817.lib.drivers.ServoMotorSubsystem;

public class Characterize extends AutoBase {

    FeedForwardCharacterization mCharacterization;

    public Characterize(String name, ServoMotorSubsystem subsystem, boolean forward) {
        mCharacterization = new FeedForwardCharacterization(subsystem, forward, new FeedForwardCharacterizationData(name));
    }

    public void routine() {
        r(mCharacterization);
    }
    
}
