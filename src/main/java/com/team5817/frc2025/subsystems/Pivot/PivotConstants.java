package com.team5817.frc2025.subsystems.Pivot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2025.Ports;
import com.team5817.frc2025.RobotConstants;
import com.team5817.lib.drivers.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team5817.lib.drivers.ServoMotorSubsystem.TalonFXConstants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Constants related to the Pivot subsystem.
 */
public final class PivotConstants {
	public static final ServoMotorSubsystemConstants kPivotServoConstants = new ServoMotorSubsystemConstants();

	static {
		kPivotServoConstants.kName = "Pivot";
		kPivotServoConstants.simIO = RobotConstants.isComp ? false : true;

		kPivotServoConstants.kMainConstants.id = Ports.ELEVATOR;
		kPivotServoConstants.kMainConstants.counterClockwisePositive = true;

		TalonFXConstants followerConstants = new TalonFXConstants();
		followerConstants.id = Ports.ELEVATOR_2;
		followerConstants.counterClockwisePositive = false;
		followerConstants.invert_sensor_phase = false;

		kPivotServoConstants.kFollowerConstants = new TalonFXConstants[]{followerConstants};

		kPivotServoConstants.kHomePosition = 0; // degrees
		kPivotServoConstants.kRotationsPerUnitDistance = 72.82 / 1.4 * 3 / 4;

		kPivotServoConstants.kMaxUnitsLimit = 2.035;
		kPivotServoConstants.kMinUnitsLimit = 0.0;

		kPivotServoConstants.kKp = 15;
		kPivotServoConstants.kKi = 0.0;
		kPivotServoConstants.kKd = 0.2;
		kPivotServoConstants.kKa = 0.0;
		kPivotServoConstants.kKs = 0.0;
		kPivotServoConstants.kKv = 0.1;
		kPivotServoConstants.kKg = 7;

		kPivotServoConstants.kCruiseVelocity = 9999.0 / kPivotServoConstants.kRotationsPerUnitDistance;
		kPivotServoConstants.kAcceleration = 300 / kPivotServoConstants.kRotationsPerUnitDistance;

		kPivotServoConstants.kMaxForwardOutput = 12.0;
		kPivotServoConstants.kMaxReverseOutput = -12.0;

		kPivotServoConstants.kEnableSupplyCurrentLimit = true;
		kPivotServoConstants.kSupplyCurrentLimit = 40; // amps
		kPivotServoConstants.kSupplyCurrentThreshold = 40; // amps
		kPivotServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

		kPivotServoConstants.kEnableStatorCurrentLimit = true;
		kPivotServoConstants.kStatorCurrentLimit = 80; // amps

		kPivotServoConstants.kFollowerOpposeMasterDirection = true;

		kPivotServoConstants.kNeutralMode = NeutralModeValue.Brake;

		kPivotServoConstants.kHomingTimeout = 0.5;
		kPivotServoConstants.kHomingOutput = -0.25;
		kPivotServoConstants.kHomingVelocityWindow = 0.1;
	}

	public static double kHomingZone = 0.1; // degrees
	public static final double kBrakeHoldPower = -0.05;
	public static final double kFollowerCurrentLimit = 40;

	public static final class State {
		public static final double SPEAKER = 0;
		public static final double AMP = 5;
		public static final double TRAP = 0;
		public static final double TRANSFER = 0;
		public static final double SHOOTING = 0;
		public static final double MAX_UP = 36.0;
		public static final double MAX_DOWN = 0;
		public static final double INTAKING = 5;
	}

	public static final InterpolatingDoubleTreeMap SpeakerAngleMap = new InterpolatingDoubleTreeMap();
	static {
		SpeakerAngleMap.put(1.1, 36.0);
		SpeakerAngleMap.put(2.11, 27.5);
		SpeakerAngleMap.put(3.08, 18.65);
		SpeakerAngleMap.put(3.71, 14.35);
		SpeakerAngleMap.put(4.67, 7.5);
		SpeakerAngleMap.put(5.5, 5.0);
	}

	public static final InterpolatingDoubleTreeMap LobAngleMap = new InterpolatingDoubleTreeMap();
	static {
		LobAngleMap.put(8.0, 20.96);
		LobAngleMap.put(12.5, 20.96);
	}

}
