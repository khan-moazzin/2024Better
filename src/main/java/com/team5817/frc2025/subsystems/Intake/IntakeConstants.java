package com.team5817.frc2025.subsystems.Intake;

import com.team5817.frc2025.Ports;
import com.team5817.frc2025.RobotConstants;
import com.team5817.lib.drivers.RollerSubsystemBasic.RollerSubsystemConstants;

public class IntakeConstants {

	/**
	 * Constants related to the Intake Roller subsystem.
	 */
	public static final class RollerConstants {
		public static final RollerSubsystemConstants kIntakeConstants = new RollerSubsystemConstants();
		public static final RollerSubsystemConstants kIndexerConstants = new RollerSubsystemConstants();

        static{
            kIntakeConstants.kName = "Intake Rollers";
            kIntakeConstants.simIO = RobotConstants.isComp? false:true;
            kIntakeConstants.kMainConstants.id = Ports.INTAKE_ROLLER;
            kIntakeConstants.kMainConstants.counterClockwisePositive = true;
			kIntakeConstants.kSupplyCurrentLimit = 40;
			kIntakeConstants.kStatorCurrentLimit = 80;
			kIntakeConstants.kEnableSupplyCurrentLimit = true;
			kIntakeConstants.kEnableStatorCurrentLimit = true;
			kIntakeConstants.kMaxForwardOutput = 12.0;
			kIntakeConstants.kMaxReverseOutput = -12.0;
			
			kIndexerConstants.kName = "Indexer Rollers";
            kIndexerConstants.simIO = RobotConstants.isComp? false:true;
            kIndexerConstants.kMainConstants.id = Ports.BOTTOM_INDEXER;
            kIndexerConstants.kMainConstants.counterClockwisePositive = false;
			kIndexerConstants.kSupplyCurrentLimit = 40;
			kIndexerConstants.kStatorCurrentLimit = 80;
			kIndexerConstants.kEnableSupplyCurrentLimit = true;
			kIndexerConstants.kEnableStatorCurrentLimit = true;
			kIndexerConstants.kMaxForwardOutput = 12.0;
			kIndexerConstants.kMaxReverseOutput = -12.0;

        }
	}

}
