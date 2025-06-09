package com.team5817.frc2025.autos.Modes;

import com.team5817.frc2025.autos.AutoBase;
import com.team5817.frc2025.autos.Actions.LambdaAction;
import com.team5817.frc2025.autos.Actions.WaitAction;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Superstructure.GoalState;

/**
 * A mode that only shoots the pre-loaded note into the speaker
 */
public class Shoot1 extends AutoBase{
    Superstructure s = Superstructure.getInstance();



    @Override
    public void routine() {
        s.setContinuousShoot(true);
        r(new WaitAction(2));
        
        r(new LambdaAction(() -> s.setGoal(GoalState.SHOOTING)));
    }
}

