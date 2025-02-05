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
	/**
	 * The period of the loop in seconds.
	 */
	public final double kPeriod;

	private boolean running_;

	private final List<Loop> loops_;
	private final Object taskRunningLock_ = new Object();
	private double timestamp_ = 0;
	private double dt_ = 0;

	/**
	 * Returns the time delta between loops.
	 *
	 * @return The time delta in seconds.
	 */
	public double dt() {
		return dt_;
	}

	/**
	 * Constructs a Looper with a specified loop time.
	 *
	 * @param loop_time The period of the loop in seconds.
	 */
	public Looper(double loop_time) {
		running_ = false;
		loops_ = new ArrayList<>();
		kPeriod = loop_time;
	}

	/**
	 * Constructs a Looper with the default loop time from Constants.
	 */
	public Looper() {
		this(Constants.kLooperDt);
	}

	/**
	 * Registers a loop to be run.
	 *
	 * @param loop The loop to be registered.
	 */
	@Override
	public synchronized void register(Loop loop) {
		synchronized (taskRunningLock_) {
			loops_.add(loop);
		}
	}

	/**
	 * Starts all registered loops.
	 */
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

	/**
	 * Updates all registered loops.
	 */
	public synchronized void update(){
		if(running_){
			for (Loop loop : loops_) {
				loop.onLoop(Timer.getTimestamp());
			}
		}
	}
}
