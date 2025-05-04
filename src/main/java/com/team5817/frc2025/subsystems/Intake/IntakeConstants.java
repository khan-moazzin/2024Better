package com.team5817.frc2025.subsystems.Intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2025.Ports;
import com.team5817.frc2025.RobotConstants;
import com.team5817.lib.drivers.RollerSubsystemBasic.RollerSubsystemConstants;
import com.team5817.lib.drivers.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team5817.lib.drivers.ServoMotorSubsystemWithCancoder.AbsoluteEncoderConstants;

public class IntakeConstants {

	/**
	 * Constants related to the Intake Roller subsystem.
	 */
	public static final class RollerConstants {
		public static final RollerSubsystemConstants kIntakeConstants = new RollerSubsystemConstants();
		public static final RollerSubsystemConstants kIndexerBottomConstants = new RollerSubsystemConstants();
		public static final RollerSubsystemConstants kIndexerSideConstants = new RollerSubsystemConstants();

        static{
            kIntakeConstants.kName = "Intake Roller";
            kIntakeConstants.simIO = RobotConstants.isComp? false:true;
            kIntakeConstants.kMainConstants.id = Ports.INTAKE_ROLLER;
            kIntakeConstants.kMainConstants.counterClockwisePositive = true;
			kIntakeConstants.kSupplyCurrentLimit = 40;
			kIntakeConstants.kStatorCurrentLimit = 80;
			kIntakeConstants.kEnableSupplyCurrentLimit = true;
			kIntakeConstants.kEnableStatorCurrentLimit = true;
			kIntakeConstants.kMaxForwardOutput = 12.0;
			kIntakeConstants.kMaxReverseOutput = -12.0;
			
			kIndexerBottomConstants.kName = "Indexer Bottom Rollers";
            kIndexerBottomConstants.simIO = RobotConstants.isComp? false:true;
            kIndexerBottomConstants.kMainConstants.id = Ports.BOTTOM_INDEXER;
            kIndexerBottomConstants.kMainConstants.counterClockwisePositive = false;
			kIndexerBottomConstants.kSupplyCurrentLimit = 40;
			kIndexerBottomConstants.kStatorCurrentLimit = 80;
			kIndexerBottomConstants.kEnableSupplyCurrentLimit = true;
			kIndexerBottomConstants.kEnableStatorCurrentLimit = true;
			kIndexerBottomConstants.kMaxForwardOutput = 12.0;
			kIndexerBottomConstants.kMaxReverseOutput = -12.0;

			kIndexerSideConstants.kName = "Indexer Side Rollers";
            kIndexerSideConstants.simIO = RobotConstants.isComp? false:true;
            kIndexerSideConstants.kMainConstants.id = Ports.SIDE_INDEXER;
            kIndexerSideConstants.kMainConstants.counterClockwisePositive = false;
			kIndexerSideConstants.kSupplyCurrentLimit = 40;
			kIndexerSideConstants.kStatorCurrentLimit = 80;
			kIndexerSideConstants.kEnableSupplyCurrentLimit = true;
			kIndexerSideConstants.kEnableStatorCurrentLimit = true;
			kIndexerSideConstants.kMaxForwardOutput = 12.0;
			kIndexerSideConstants.kMaxReverseOutput = -12.0;
        }
	}

	/**
	 * Constants related to the Intake Deploy subsystem.
	 */
	public static final class DeployConstants {
		public static final ServoMotorSubsystemConstants kDeployServoConstants = new ServoMotorSubsystemConstants();

		public static final AbsoluteEncoderConstants kDeployEncoderConstants = new AbsoluteEncoderConstants();

		static {

			kDeployServoConstants.kName = "Deploy";
			kDeployServoConstants.simIO = RobotConstants.isComp? false:true;

			kDeployServoConstants.kMainConstants.id = Ports.INTAKE_PIVOT;
			kDeployServoConstants.kMainConstants.counterClockwisePositive = false;
			

			kDeployServoConstants.kHomePosition = 0; // degrees
			kDeployServoConstants.kRotationsPerUnitDistance = (1.0 / 360.0)*20;

			kDeployServoConstants.kMaxUnitsLimit = 100000;
			kDeployServoConstants.kMinUnitsLimit = -100000;

			kDeployServoConstants.kKp = 3.8125	;
			kDeployServoConstants.kKi = 0.0;   
			kDeployServoConstants.kKd = 0;
			kDeployServoConstants.kKa = 0;
			kDeployServoConstants.kKs = 0;
			kDeployServoConstants.kKv = .5;
			kDeployServoConstants.kKg = 0.265625;

 
			kDeployServoConstants.kCruiseVelocity = 32*360*1/20; 
			kDeployServoConstants.kAcceleration = 32*360*1/20; 

			kDeployServoConstants.kMaxForwardOutput = 12.0;
			kDeployServoConstants.kMaxReverseOutput = -12.0;

			kDeployServoConstants.kEnableSupplyCurrentLimit = true;
			kDeployServoConstants.kSupplyCurrentLimit = 80; // amps
			kDeployServoConstants.kSupplyCurrentThreshold = 80; // amps
			kDeployServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

			kDeployServoConstants.kEnableStatorCurrentLimit = true;
			kDeployServoConstants.kStatorCurrentLimit = 80; // amps

			kDeployServoConstants.kNeutralMode = NeutralModeValue.Brake;

			kDeployEncoderConstants.rotor_to_sensor_ratio = 20;
			kDeployEncoderConstants.remote_encoder_port = Ports.INTAKE_CANCODER;

			kDeployServoConstants.kHomingOutput = -.3;
			kDeployServoConstants.kHomingTimeout = 0.2;
			kDeployServoConstants.kHomingVelocityWindow = 5;
		}

	}
}
