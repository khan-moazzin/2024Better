package com.team5817.frc2024.autos.Modes;

import com.team5817.frc2024.autos.AutoBase;
import com.team5817.frc2024.autos.Actions.TrajectoryAction;
import com.team5817.frc2024.autos.TrajectoryLibrary.l;
import com.team5817.frc2024.subsystems.Superstructure;
import com.team5817.frc2024.subsystems.Drive.Drive;
import com.team5817.lib.motion.TrajectorySet;

public class TestPathMode extends AutoBase{

	private Superstructure s = Superstructure.getInstance();
	private Drive d = Drive.getInstance();
    private TrajectorySet t;

	public TestPathMode() {

        t = new TrajectorySet(
            l.testPath1,
            l.testPath2
        );

	}

	// spotless:off
	@Override
	public void routine() {
        runAction(new TrajectoryAction(t.next()));
        
        System.out.println("Finished auto!");
	}
	// spotless:on
   
}
