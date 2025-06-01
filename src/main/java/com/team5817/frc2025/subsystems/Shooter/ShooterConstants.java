package com.team5817.frc2025.subsystems.Shooter;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team5817.frc2025.Ports;
import com.team5817.frc2025.RobotConstants;
import com.team5817.lib.drivers.RollerSubsystemBasic.RollerSubsystemConstants;
import com.team5817.lib.drivers.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    
	/**
	 * Constants related to the End Effector Wrist subsystem.
	 */
	public static final class ShooterMotorConstants {

		public static final RollerSubsystemConstants kShooterMotor1Constants = new RollerSubsystemConstants();
        public static final RollerSubsystemConstants kShooterMotor2Constants = new RollerSubsystemConstants();
        public static final RollerSubsystemConstants[] kShooterConstants = {
     
        };

		static {
            kShooterMotor1Constants.kName = "ShooterMotor1";
            kShooterMotor1Constants.simIO = RobotConstants.isComp ? false : true;
            kShooterMotor1Constants.id = Ports.SHOOTER_1;
            kShooterMotor1Constants.kStatorCurrentLimit = 100; // amps
            kShooterMotor1Constants.kSupplyCurrentLimit = 30; // amps
            kShooterMotor1Constants.kEnableSupplyCurrentLimit = true;
            kShooterMotor1Constants.kEnableStatorCurrentLimit = true;
            kShooterMotor1Constants.kMaxForwardOutput = 12.0;
            kShooterMotor1Constants.kMaxReverseOutput = -12.0;
            kShooterMotor1Constants.kNeutralMode = NeutralModeValue.Brake;
     
		
		
			kShooterMotor2Constants.kName = "ShooterMotor2";
            kShooterMotor2Constants.simIO = RobotConstants.isComp ? false : true;
            kShooterMotor2Constants.id = Ports.SHOOTER_2;
            kShooterMotor2Constants.kStatorCurrentLimit = 100; // amps
            kShooterMotor2Constants.kSupplyCurrentLimit = 30; // amps
            kShooterMotor2Constants.kEnableSupplyCurrentLimit = true;
            kShooterMotor2Constants.kEnableStatorCurrentLimit = true;
            kShooterMotor2Constants.kMaxForwardOutput = 12.0;
            kShooterMotor2Constants.kMaxReverseOutput = -12.0;
            kShooterMotor2Constants.kNeutralMode = NeutralModeValue.Brake;


		
		
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
    
    public static final double kShotTime = 1.2;

    
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> SPEAKER_VELOCITY_TREE_MAP = new InterpolatingTreeMap<>();

    static {
        SPEAKER_VELOCITY_TREE_MAP.put(new InterpolatingDouble(1.0), new InterpolatingDouble(.4));
        SPEAKER_VELOCITY_TREE_MAP.put(new InterpolatingDouble(3.0), new InterpolatingDouble(1.0));
    }
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> SPIN_TREE_MAP = new InterpolatingTreeMap<>();
    static{
        SPIN_TREE_MAP.put(new InterpolatingDouble(1.0), new InterpolatingDouble(1.0));
        SPIN_TREE_MAP.put(new InterpolatingDouble(3.0), new InterpolatingDouble(.5));
    }
    
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> LOB_VELOCITY_TREE_MAP = new InterpolatingTreeMap<>();
    static {
        LOB_VELOCITY_TREE_MAP.put(new InterpolatingDouble(8.25), new InterpolatingDouble(0.6));
        LOB_VELOCITY_TREE_MAP.put(new InterpolatingDouble(16.0), new InterpolatingDouble(.6));
    }
    public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> SHOT_TRAVEL_TIME_TREE_MAP = new InterpolatingTreeMap<>();
    static {
        SHOT_TRAVEL_TIME_TREE_MAP.put(new InterpolatingDouble(1.0), new InterpolatingDouble(0.0003));
        SHOT_TRAVEL_TIME_TREE_MAP.put(new InterpolatingDouble(6.0), new InterpolatingDouble(0.0045));
    }
	
}
