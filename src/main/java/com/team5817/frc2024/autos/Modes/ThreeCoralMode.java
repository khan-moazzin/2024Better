package com.team5817.frc2024.autos.Modes;

import com.team5817.frc2024.autos.AutoBase;
import com.team5817.frc2024.autos.TrajectoryLibrary.l;
import com.team5817.frc2024.subsystems.Superstructure;
import com.team5817.frc2024.subsystems.Drive.Drive;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;

public class ThreeCoralMode extends AutoBase {


	private Superstructure s = Superstructure.getInstance();
	private Drive d = Drive.getInstance();

	public ThreeCoralMode() {

        TrajectorySet t = new TrajectorySet(
            l.leftStartToCloseScore,
            l.rightStartToCloseScore
        );

	}

	// spotless:off
	@Override
	public void routine() {

        
        System.out.println("Finished auto!");
	}
	// spotless:on
}