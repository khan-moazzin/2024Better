// package com.team5817.frc2025.autos.Actions;


// import com.team5817.frc2025.subsystems.Superstructure;

// import edu.wpi.first.wpilibj.Timer;

// public class WaitForSuperstructureAction implements Action {
// 	private double mTimeToWait;
// 	private double mStartTime;

// 	public WaitForSuperstructureAction(double timeToWait) {
// 		mTimeToWait = timeToWait;
// 	}

// 	@Override
// 	public boolean isFinished() {
// 		return Timer.getFPGATimestamp() - mStartTime >= mTimeToWait || Superstructure.getInstance().requestsCompleted();
// 	}


// 	@Override
// 	public void start() {
// 	}

// 	@Override
// 	public void update() {}

// 	@Override
// 	public void done() {}
// }