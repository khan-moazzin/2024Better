package com.team5817.frc2025;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;

/**
 * Constants class holds all the robot-wide numerical or boolean constants.
 * This class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 */
public class RobotConstants {

	// Disables extra smart dashboard outputs that slow down the robot
	public static final boolean disableExtraTelemetry = false;

	// robot loop time
	public static final double kLooperDt = 0.02;

	/* Control Board */
	public static final double kJoystickThreshold = 0.2;
	public static final int kButtonGamepadPort = 1;

	public static final double stickDeadband = 0.1;

	public static final double specializedVisionTimeout = 5;

	// Timeout constants
	public static final double kLongCANTimeoutS = 0.1;
	public static final double kCANTimeoutS = .01;
	public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.04, 1), Math.pow(0.04, 1));
	public static final Matrix<N2, N1> kLocalMeasurementStdDevs = VecBuilder.fill(Math.pow(0.001, 1), Math.pow(0.001, 1));
	public static final double[][] fundamentalMatrix = {
			{ 0.0, 0.0, 0.0 },
			{ 0.0, 0.0, 0.0 },
			{ 0.0, 0.0, 0.0 }
	};

	public static final double kBumberSideLength = Units.inchesToMeters(36.125-3);


    public static final double kDefaultDistanceToReef = 3;
	
	public static final boolean isComp = isComp();
	private static boolean isComp(){
		final Path commentPath = Path.of("/etc/machine-info");
		try {
			var comment = Files.readString(commentPath);
			return comment.contains("Comp");
		} catch (IOException e) {
			System.out.println(e);
			return false;
		}
		}
}
