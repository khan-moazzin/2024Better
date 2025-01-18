// package com.team5817.frc2025.autos.Modes;

// import com.team5817.frc2025.autos.AutoBase;
// import com.team5817.frc2025.autos.Actions.TrajectoryAction;
// import com.team5817.frc2025.autos.TrajectoryLibrary.l;
// import com.team5817.frc2025.subsystems.Superstructure;
// import com.team5817.frc2025.subsystems.Drive.Drive;
// import com.team5817.lib.motion.Trajectory;
// import com.team5817.lib.motion.TrajectorySet;

// public class ThreeCoralMode extends AutoBase {


// 	private Superstructure s = Superstructure.getInstance();
// 	private Drive d = Drive.getInstance();

// 	private TrajectorySet t;

// 	public ThreeCoralMode() {

//         t = new TrajectorySet(
//             l.sideStartToCloseScore,
//             l.sideStartToCloseScore
//         );
// 		// if()
// 		// t.mirror();

// 	}

// 	// spotless:off
// 	@Override
// 	public void routine() {
// 		runAction(new TrajectoryAction(t.next()));
        
//         System.out.println("Finished auto!");
// 	}
// 	// spotless:on
// }