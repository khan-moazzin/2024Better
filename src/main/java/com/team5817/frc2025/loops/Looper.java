package com.team5817.frc2025.loops;

import com.team5817.frc2025.Constants;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List object. They are started when the robot
 * powers up and stopped after the match.
 */
public class Looper implements ILooper {
	public final double kPeriod;

	private boolean running_;

	private final List<Loop> loops_;
	private final Object taskRunningLock_ = new Object();
	private double timestamp_ = 0;
	private double dt_ = 0;

	public double dt() {
		return dt_;
	}



	public Looper(double loop_time) {
		running_ = false;
		loops_ = new ArrayList<>();
		kPeriod = loop_time;
	}

	public Looper() {
		this(Constants.kLooperDt);
	}

	@Override
	public synchronized void register(Loop loop) {
		synchronized (taskRunningLock_) {
			loops_.add(loop);
		}
	}

	public synchronized void start() {
		if (!running_) {
			System.out.println("Starting loops");
			synchronized (taskRunningLock_) {
				timestamp_ = Timer.getTimestamp();
				for (Loop loop : loops_) {
					loop.onStart(timestamp_);
				}
				running_ = true;
			}
		}
	}
	public synchronized void update(){
		if(running_){
			for (Loop loop : loops_) {
				loop.onLoop(Timer.getTimestamp());
			}
		}
	}
}
