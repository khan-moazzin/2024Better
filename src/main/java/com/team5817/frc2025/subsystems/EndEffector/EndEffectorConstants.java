package com.team5817.frc2025.subsystems.EndEffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2025.Ports;
import com.team5817.frc2025.RobotConstants;
import com.team5817.lib.drivers.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team5817.lib.drivers.ServoMotorSubsystemWithCancoder.AbsoluteEncoderConstants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class EndEffectorConstants {
    
	/**
	 * Constants related to the End Effector Wrist subsystem.
	 */
	public static final class EndEffectorWristConstants {
		// 115.93
		// 7.92
		public static final ServoMotorSubsystemConstants kWristServoConstants = new ServoMotorSubsystemConstants();

		public static final AbsoluteEncoderConstants kWristEncoderConstants = new AbsoluteEncoderConstants();

		static {
			kWristServoConstants.kName = "EndEffectorWrist";

			kWristServoConstants.simIO = RobotConstants.isComp? false:true;

			kWristServoConstants.kMainConstants.id = Ports.ENDEFFECTOR_WRIST;
			kWristServoConstants.kMainConstants.counterClockwisePositive = false;

			kWristServoConstants.kHomePosition = -89; // degrees
			kWristServoConstants.kRotationsPerUnitDistance = (1/360.0) * 10;

			kWristServoConstants.kMaxUnitsLimit = 200-89;
			kWristServoConstants.kMinUnitsLimit = -89;

			kWristServoConstants.kKp = 100;
			kWristServoConstants.kKi = 0;
			kWristServoConstants.kKd = 0.1;
			kWristServoConstants.kKa = 0.0;
			kWristServoConstants.kKs = 0.04;
			kWristServoConstants.kKv = 0;
			kWristServoConstants.kKg = 3;
			kWristServoConstants.kGravityType = GravityTypeValue.Arm_Cosine;


			kWristServoConstants.kCruiseVelocity = 160000; // degrees / s
			kWristServoConstants.kAcceleration = 9000.0; // degrees / s^2

			kWristServoConstants.kMaxForwardOutput = 12.0;
			kWristServoConstants.kMaxReverseOutput = -12.0;
			

			kWristServoConstants.kEnableSupplyCurrentLimit = true;
			kWristServoConstants.kSupplyCurrentLimit = 40; // amps
			kWristServoConstants.kSupplyCurrentThreshold = 40; // amps
			kWristServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

			kWristServoConstants.kEnableStatorCurrentLimit = true;
			kWristServoConstants.kStatorCurrentLimit = 30; // amps

			kWristServoConstants.kNeutralMode = NeutralModeValue.Brake;

			kWristServoConstants.kHomingTimeout = .5;
			kWristServoConstants.kHomingOutput = -.25;
			kWristServoConstants.kHomingVelocityWindow = 1;
		}
		public static final InterpolatingDoubleTreeMap kHighOffsetMap = new InterpolatingDoubleTreeMap();
		static {
			kHighOffsetMap.put(-.11, 27.0);
			kHighOffsetMap.put(0.0, 0.0);
		}
		public static final InterpolatingDoubleTreeMap kMidOffsetMap = new InterpolatingDoubleTreeMap();
		static {
			kMidOffsetMap.put(-.11, 0.0);
			kMidOffsetMap.put(0.0, 0.0);
		}
	}

	/**
	 * Constants related to the End Effector Roller subsystem.
	 */
	public static final class EndEffectorRollerConstants {
		public static TalonFXConfiguration RollerFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyCurrentLimit = 30.0;

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 100.0;

			config.Voltage.PeakForwardVoltage = 12.0;
			config.Voltage.PeakReverseVoltage = -12.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			return config;
		}
	}
}
