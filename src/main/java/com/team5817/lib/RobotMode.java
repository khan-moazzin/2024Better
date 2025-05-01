package com.team5817.lib;

public class RobotMode {
    /**
	 * Enum representing the different modes the robot can operate in.
	 */
	public enum Mode {
		SIM,
		REPLAY,
		REAL
	}

	public static Mode mode = Mode.SIM;//Sim or Replay, Real is auto set for real robot
}
