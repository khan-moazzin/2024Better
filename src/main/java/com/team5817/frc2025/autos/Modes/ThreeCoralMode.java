package com.team5817.frc2025.autos.Modes;

import com.team5817.frc2025.Constants;
import com.team5817.frc2025.Robot;
import com.team5817.frc2025.autos.AutoBase;
import com.team5817.frc2025.autos.Actions.TrajectoryAction;
import com.team5817.frc2025.autos.TrajectoryLibrary.l;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.lib.motion.TrajectorySet;

public class ThreeCoralMode extends AutoBase {


	private Superstructure s = Superstructure.getInstance();
	private Drive d = Drive.getInstance();
	private TrajectorySet t;

	public ThreeCoralMode() {

        t = new TrajectorySet(
            l.leftToHuman,
			l.humanToApp
        );
		// if()j

	}

	// spotless:off
	@Override
	public void routine() {
		System.out.println("ran auto");
        if(!Robot.isReal() && Constants.mode == Constants.Mode.SIM){
            mSim.setSimulationWorldPose(t.initalPose().wpi());
		}
		
		r(new TrajectoryAction(t.next()));
		r(new TrajectoryAction(t.next()));
		// runAction(new WaitAction(1));

        
        System.out.println("Finished auto!");
	}
}