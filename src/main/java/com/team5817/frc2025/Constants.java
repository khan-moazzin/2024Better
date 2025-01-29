package com.team5817.frc2025;

import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.team5817.frc2025.subsystems.vision.VisionDeviceConstants;
import com.team5817.lib.drivers.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team5817.lib.drivers.ServoMotorSubsystemWithCancoder.AbsoluteEncoderConstants;
import com.team5817.lib.swerve.SwerveModule.SwerveModuleConstants;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.motion.MotionProfileConstraints;
import com.team254.lib.swerve.SwerveDriveKinematics;
import com.team254.lib.swerve.SwerveKinematicLimits;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class Constants {

	public enum Mode{
		SIM,
		REPLAY
	}
	public static Mode mode = Mode.SIM;

	// Disables extra smart dashboard outputs that slow down the robot
	public static final boolean disableExtraTelemetry = false;

	// robot loop time
	public static final double kLooperDt = 0.02;

	/* Control Board */
	public static final double kJoystickThreshold = 0.2;
	public static final int kButtonGamepadPort = 1;

	public static final double stickDeadband = 0.1;

	// Timeout constants
	public static final double kLongCANTimeoutS = 0.1;
	public static final double kCANTimeoutS = .01;
    public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.04, 1), Math.pow(0.04, 1));
    public static final Matrix<N2, N1> kLocalMeasurementStdDevs = VecBuilder.fill(Math.pow(0.01, 1), Math.pow(0.01, 1));
    public static final double[][] fundamentalMatrix = 
	{
	{0.0,0.0,0.0},
	{0.0,0.0,0.0},
	{0.0,0.0,0.0}
};

    public static final double kBumberSideLength = Units.inchesToMeters(29);

    public static final boolean kSubsytemSim = true;//Forces sim IO een if robot is real for partial robot(DB)


	public static final class SwerveConstants {

		public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

		/* Drivetrain Constants */
		public static final double trackWidth = Units.inchesToMeters(29);
		public static final double wheelBase = Units.inchesToMeters(29);

		public static final double wheelDiameter = Units.inchesToMeters(4.00);
		public static final double wheelCircumference = wheelDiameter * Math.PI;

		public static final double driveGearRatio = 6.25; 
		public static final double angleGearRatio = 15.43; 

		public static final Translation2d[] swerveModuleLocations = {
			new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
			new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
			new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
			new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
		};
		public static final edu.wpi.first.math.geometry.Translation2d[] swerveModuleLocationsWpi;

		static {
			swerveModuleLocationsWpi = new edu.wpi.first.math.geometry.Translation2d[4];
			for(int i = 0; i< swerveModuleLocations.length; i++){
				swerveModuleLocationsWpi[i] = swerveModuleLocations[i].wpi();
			}
		}
		public static final RobotConfig mRobotConfig = new RobotConfig(43, 1, new ModuleConfig(Units.inchesToMeters(2),6,.85,DCMotor.getKrakenX60(1),40,1), swerveModuleLocationsWpi);

		public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(swerveModuleLocations);

		public static final edu.wpi.first.math.geometry.Translation2d[] wpiModuleLocations = {
			new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
			new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, trackWidth / 2.0),
			new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
			new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, -trackWidth / 2.0)
		};

		public static final edu.wpi.first.math.kinematics.SwerveDriveKinematics kWpiKinematics =
				new edu.wpi.first.math.kinematics.SwerveDriveKinematics(wpiModuleLocations);

		/* Swerve Profiling Values */
		public static final double maxSpeed = 5; // meters per second
		public static final double maxAcceleration = 6; // meters per second
		public static final double maxAngularVelocity = 11.5;
		public static final double maxAngularAcceleration = maxAcceleration /
            Math.hypot(wheelBase / 2.0, trackWidth / 2.0);

		public static final double kV = 12 * Math.PI * wheelDiameter / (driveGearRatio * maxSpeed);
		public static final double maxAutoSpeed = maxSpeed * 0.85; // Max out at 85% to ensure attainable speeds

		/* Motor Inverts */
		public static final boolean driveMotorInvert = false;
		public static final boolean angleMotorInvert = true;

		/* Angle Encoder Invert */
		public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

		/* Controller Invert */
		public static final boolean invertYAxis = false;
		public static final boolean invertRAxis = false;
		public static final boolean invertXAxis = false;

		/* Heading Controller */

		// Stabilize Heading PID Values
		public static final double kStabilizeSwerveHeadingKp = 10.0;
		public static final double kStabilizeSwerveHeadingKi = 0.0;
		public static final double kStabilizeSwerveHeadingKd = 0.3;
		public static final double kStabilizeSwerveHeadingKf = 2.0;

		// Snap Heading PID Values
		public static final double kSnapSwerveHeadingKp = 10.0;
		public static final double kSnapSwerveHeadingKi = 0.0;
		public static final double kSnapSwerveHeadingKd = 0.6;
		public static final double kSnapSwerveHeadingKf = 1.0;
		
		public static final double kTrajectoryDeadband = .01;

		public static final SwerveKinematicLimits kSwerveKinematicLimits = new SwerveKinematicLimits();

		static {
			kSwerveKinematicLimits.kMaxDriveVelocity = maxSpeed;
			kSwerveKinematicLimits.kMaxDriveAcceleration = 40;
			kSwerveKinematicLimits.kMaxSteeringVelocity = maxAngularVelocity;
		}
		
		public static final SwerveKinematicLimits kSwerveUncappedKinematicLimits = new SwerveKinematicLimits();

		static {
			kSwerveUncappedKinematicLimits.kMaxDriveVelocity = maxSpeed;
			kSwerveUncappedKinematicLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
			kSwerveUncappedKinematicLimits.kMaxSteeringVelocity = maxAngularVelocity;
		}

		public static final double kAutoAlignAllowableDistance = 2.0; //Meters
	
	    public static final MotionProfileConstraints kPositionMotionProfileConstraints = new MotionProfileConstraints(
             maxSpeed,
        	 -maxSpeed,
             maxAcceleration);

		public static final MotionProfileConstraints kHeadingMotionProfileConstraints = new MotionProfileConstraints(
            5,
            -5,
            maxAngularAcceleration*0.5);


		/*** MODULE SPECIFIC CONSTANTS ***/
		/* Front Left Module - Module 0 */
		public static final class Mod0 {
			public static final double compAngleOffset = 180;

			public static SwerveModuleConstants SwerveModuleConstants() {
				return new SwerveModuleConstants(
						Ports.FL_DRIVE.getDeviceNumber(),
						Ports.FL_ROTATION.getDeviceNumber(),
						compAngleOffset);
			}
		}

		/* Front Right Module - Module 1 */
		public static final class Mod1 {
			public static final double compAngleOffset =0;

			public static SwerveModuleConstants SwerveModuleConstants() {
				return new SwerveModuleConstants(
						Ports.FR_DRIVE.getDeviceNumber(),
						Ports.FR_ROTATION.getDeviceNumber(),
						compAngleOffset);
			}
		}

		/* Back Left Module - Module 2 */
		public static final class Mod2 {
			public static final double compAngleOffset = 180;

			public static SwerveModuleConstants SwerveModuleConstants() {
				return new SwerveModuleConstants(
						Ports.BL_DRIVE.getDeviceNumber(),
						Ports.BL_ROTATION.getDeviceNumber(),
						compAngleOffset);
			}
		}

		/* Back Right Module - Module 3 */
		public static final class Mod3 {
			public static final double compAngleOffset = 0;

			public static SwerveModuleConstants SwerveModuleConstants() {
				return new SwerveModuleConstants(
						Ports.BR_DRIVE.getDeviceNumber(),
						Ports.BR_ROTATION.getDeviceNumber(),
						compAngleOffset);
			}
		}

		public static TalonFXConfiguration DriveFXConfig(boolean inverse) {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.Slot0.kP = 0.030 * 12.0;
			config.Slot0.kI = 0.0;
			config.Slot0.kD = 0.000001 * 12.0;
			config.Slot0.kS = 0.1;
			config.Slot0.kV = 12 * Math.PI * wheelDiameter / (driveGearRatio * maxSpeed);

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 110;

			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyCurrentLimit = 90;

			config.Voltage.PeakForwardVoltage = 12.0;
			config.Voltage.PeakReverseVoltage = -12.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			config.MotorOutput.Inverted = inverse ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;


			config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
			config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.25;
			return config;
		}

		public static TalonFXConfiguration AzimuthFXConfig(boolean inverse) {
			TalonFXConfiguration config = new TalonFXConfiguration();
			

			config.Slot0.kP = 1.0005;
			config.Slot0.kI = 0.0;
			config.Slot0.kD = 0.0004;
			config.Slot0.kS = 0.0;
			config.Slot0.kV = 0.0;

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 80;

			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyCurrentLimit = 60;

			config.Voltage.PeakForwardVoltage = 12.0;
			config.Voltage.PeakReverseVoltage = -12.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			config.MotorOutput.Inverted = inverse ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

			return config;
		}

		public static CANcoderConfiguration swerveCancoderConfig() {
			CANcoderConfiguration CANCoderConfig = new com.ctre.phoenix6.configs.CANcoderConfiguration();
			CANCoderConfig.MagnetSensor.SensorDirection = Constants.SwerveConstants.canCoderInvert;
			return CANCoderConfig;
		}

		public static final double kCancoderBootAllowanceSeconds = 10.0;
	}

	public static final class PoseEstimatorConstants {
		public record CameraConfig(Pose3d offset, String config) {}
		;

		public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.2, 1), Math.pow(0.2, 1));
		public static final Matrix<N2, N1> kLocalMeasurementStdDevs =
				VecBuilder.fill(Math.pow(0.01, 1), Math.pow(0.01, 1));


	public static VisionDeviceConstants kDomVisionDevice = new VisionDeviceConstants(); // dot 13
	public static VisionDeviceConstants kSubVisionDevice = new VisionDeviceConstants(); // dot 12

	public static List<Integer> redTagIDFilters;
	public static List<Integer> blueTagIDFilters;

	static {
		redTagIDFilters = List.of();
		blueTagIDFilters = List.of();

		kDomVisionDevice.kTableName = "limelight-Dom";
		kDomVisionDevice.kRobotToCamera = new Transform3d(Units.inchesToMeters(3.071), Units.inchesToMeters(7.325),Units.inchesToMeters(0), 
				new Rotation3d(0,0,0));

		kSubVisionDevice.kTableName = "limelight-Sub";
		kSubVisionDevice.kRobotToCamera = new Transform3d(Units.inchesToMeters(3.071), Units.inchesToMeters(-7.325), Units.inchesToMeters(0),
				new Rotation3d(0,0,0));//TODO set these to correct values

	}

}

	public static final class IntakeDeployConstants {
		// 115.93
		// 7.92
		public static final ServoMotorSubsystemConstants kDeployServoConstants = new ServoMotorSubsystemConstants();

		public static final AbsoluteEncoderConstants kDeployEncoderConstants = new AbsoluteEncoderConstants();

		static {
			kDeployServoConstants.kName = "Deploy";

			kDeployServoConstants.kMainConstants.id = Ports.INTAKE_PIVOT;
			kDeployServoConstants.kMainConstants.counterClockwisePositive = false;

			kDeployServoConstants.kHomePosition = 0; // degrees
			kDeployServoConstants.kRotationsPerUnitDistance = 1;

			kDeployServoConstants.kMaxUnitsLimit = 1.5;
			kDeployServoConstants.kMinUnitsLimit = 0.0;

			kDeployServoConstants.kKp = 3.0;
			kDeployServoConstants.kKi = 0.0;
			kDeployServoConstants.kKd = 0.0;
			kDeployServoConstants.kKa = 0.0;
			kDeployServoConstants.kKs = 0.2;
			kDeployServoConstants.kKg = 0.2;

			kDeployServoConstants.kCruiseVelocity = 400.0; // degrees / s
			kDeployServoConstants.kAcceleration = 10000.0; // degrees / s^2

			kDeployServoConstants.kMaxForwardOutput = 12.0;
			kDeployServoConstants.kMaxReverseOutput = -12.0;

			kDeployServoConstants.kEnableSupplyCurrentLimit = true;
			kDeployServoConstants.kSupplyCurrentLimit = 40; // amps
			kDeployServoConstants.kSupplyCurrentThreshold = 40; // amps
			kDeployServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

			kDeployServoConstants.kEnableStatorCurrentLimit = true;
			kDeployServoConstants.kStatorCurrentLimit = 80; // amps

			kDeployServoConstants.kNeutralMode = NeutralModeValue.Brake;

			kDeployEncoderConstants.encoder_type = FeedbackSensorSourceValue.FusedCANcoder;
			kDeployEncoderConstants.remote_encoder_port = Ports.INTAKE_CANCODER;
			kDeployEncoderConstants.rotor_rotations_per_output = 314.0;
			kDeployEncoderConstants.remote_encoder_offset = 0;
		}

		public static double kHomingZone = 7.0; // degrees
		public static double kHomingTimeout = 0.2; // seconds
		public static double kHomingVelocityWindow = 5.0; // "units" / secon
		public static double kHomingOutput = 4.0; // volts

	}

	public static final class IntakeRollerConstants {
		public static TalonFXConfiguration RollerFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyCurrentLimit = 40.0;

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 80.0;

			config.Voltage.PeakForwardVoltage = 12.0;
			config.Voltage.PeakReverseVoltage = -12.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			return config;
		}
	}

	public static final class ElevatorConstants {
		// 115.93
		// 7.92
		public static final ServoMotorSubsystemConstants kElevatorServoConstants = new ServoMotorSubsystemConstants();

		static {
			kElevatorServoConstants.kName = "Elevator";

			kElevatorServoConstants.kMainConstants.id = Ports.ELEVATOR;
			kElevatorServoConstants.kMainConstants.counterClockwisePositive = false;

			kElevatorServoConstants.kHomePosition = 0; // degrees
			kElevatorServoConstants.kRotationsPerUnitDistance = 1;

			kElevatorServoConstants.kMaxUnitsLimit = 128.1;
			kElevatorServoConstants.kMinUnitsLimit = 0.0;

			kElevatorServoConstants.kKp = 3.0;
			kElevatorServoConstants.kKi = 0.0;
			kElevatorServoConstants.kKd = 0.0;
			kElevatorServoConstants.kKa = 0.0;
			kElevatorServoConstants.kKs = 0.2;
			kElevatorServoConstants.kKg = 0.2;

			kElevatorServoConstants.kCruiseVelocity = 400.0; // degrees / s
			kElevatorServoConstants.kAcceleration = 10000.0; // degrees / s^2

			kElevatorServoConstants.kMaxForwardOutput = 12.0;
			kElevatorServoConstants.kMaxReverseOutput = -12.0;

			kElevatorServoConstants.kEnableSupplyCurrentLimit = true;
			kElevatorServoConstants.kSupplyCurrentLimit = 40; // amps
			kElevatorServoConstants.kSupplyCurrentThreshold = 40; // amps
			kElevatorServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

			kElevatorServoConstants.kEnableStatorCurrentLimit = true;
			kElevatorServoConstants.kStatorCurrentLimit = 80; // amps

			kElevatorServoConstants.kNeutralMode = NeutralModeValue.Brake;
		}

		public static final double kCoralClearHeight = 0.15; // rotations
	}

	public static final class ClimbConstants {
		// 115.93
		// 7.92
		public static final ServoMotorSubsystemConstants kClimbServoConstants = new ServoMotorSubsystemConstants();

		static {
			kClimbServoConstants.kName = "Climb";

			kClimbServoConstants.kMainConstants.id = Ports.ELEVATOR;
			kClimbServoConstants.kMainConstants.counterClockwisePositive = false;

			kClimbServoConstants.kHomePosition = 0; // degrees
			kClimbServoConstants.kRotationsPerUnitDistance = (1.0 / 360.0) * (45.0 / 1.0);

			kClimbServoConstants.kMaxUnitsLimit = 128.1;
			kClimbServoConstants.kMinUnitsLimit = 0.0;

			kClimbServoConstants.kKp = 3.0;
			kClimbServoConstants.kKi = 0.0;
			kClimbServoConstants.kKd = 0.0;
			kClimbServoConstants.kKa = 0.0;
			kClimbServoConstants.kKs = 0.2;
			kClimbServoConstants.kKg = 0.2;

			kClimbServoConstants.kCruiseVelocity = 400.0; // degrees / s
			kClimbServoConstants.kAcceleration = 10000.0; // degrees / s^2

			kClimbServoConstants.kMaxForwardOutput = 12.0;
			kClimbServoConstants.kMaxReverseOutput = -12.0;

			kClimbServoConstants.kEnableSupplyCurrentLimit = true;
			kClimbServoConstants.kSupplyCurrentLimit = 40; // amps
			kClimbServoConstants.kSupplyCurrentThreshold = 40; // amps
			kClimbServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

			kClimbServoConstants.kEnableStatorCurrentLimit = true;
			kClimbServoConstants.kStatorCurrentLimit = 80; // amps

			kClimbServoConstants.kNeutralMode = NeutralModeValue.Brake;
		}

	}


	public static final class EndEffectorWristConstants {
		// 115.93
		// 7.92
		public static final ServoMotorSubsystemConstants kWristServoConstants = new ServoMotorSubsystemConstants();

		public static final AbsoluteEncoderConstants kWristEncoderConstants = new AbsoluteEncoderConstants();

		static {
			kWristServoConstants.kName = "Wrist";

			kWristServoConstants.kMainConstants.id = Ports.ENDEFFECTOR_WRIST;
			kWristServoConstants.kMainConstants.counterClockwisePositive = false;

			kWristServoConstants.kHomePosition = 0; // degrees
			kWristServoConstants.kRotationsPerUnitDistance = (1.0 / 360.0) * (45.0 / 1.0);

			kWristServoConstants.kMaxUnitsLimit = 128.1;
			kWristServoConstants.kMinUnitsLimit = 0.0;

			kWristServoConstants.kKp = 3.0;
			kWristServoConstants.kKi = 0.0;
			kWristServoConstants.kKd = 0.0;
			kWristServoConstants.kKa = 0.0;
			kWristServoConstants.kKs = 0.2;
			kWristServoConstants.kKg = 0.2;

			kWristServoConstants.kCruiseVelocity = 4000.0; // degrees / s
			kWristServoConstants.kAcceleration = 100000.0; // degrees / s^2

			kWristServoConstants.kMaxForwardOutput = 12.0;
			kWristServoConstants.kMaxReverseOutput = -12.0;

			kWristServoConstants.kEnableSupplyCurrentLimit = true;
			kWristServoConstants.kSupplyCurrentLimit = 40; // amps
			kWristServoConstants.kSupplyCurrentThreshold = 40; // amps
			kWristServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

			kWristServoConstants.kEnableStatorCurrentLimit = true;
			kWristServoConstants.kStatorCurrentLimit = 80; // amps

			kWristServoConstants.kNeutralMode = NeutralModeValue.Brake;

			kWristEncoderConstants.encoder_type = FeedbackSensorSourceValue.FusedCANcoder;
			kWristEncoderConstants.remote_encoder_port = Ports.INTAKE_CANCODER;
			kWristEncoderConstants.rotor_rotations_per_output = 314.0;
			kWristEncoderConstants.remote_encoder_offset = 0;
		}

	}

	public static final class EndEffectorRollerConstants {
		public static TalonFXConfiguration RollerFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyCurrentLimit = 40.0;

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 80.0;

			config.Voltage.PeakForwardVoltage = 12.0;
			config.Voltage.PeakReverseVoltage = -12.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			return config;
		}
	}


}
