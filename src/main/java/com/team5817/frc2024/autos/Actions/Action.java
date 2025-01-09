package com.team5817.frc2024.autos.Actions;


public interface Action {
	boolean isFinished();
	void update();
	void done();
	void start();
}
