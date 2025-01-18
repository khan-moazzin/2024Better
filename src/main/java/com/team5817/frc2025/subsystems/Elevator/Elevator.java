// package com.team5817.frc2025.subsystems.Elevator;

// import com.team5817.frc2025.Constants.ElevatorConstants;
// import com.team5817.frc2025.loops.ILooper;
// import com.team5817.frc2025.loops.Loop;
// import com.team5817.lib.Util;
// import com.team5817.lib.drivers.ServoMotorSubsystem;
// import com.team5817.lib.requests.Request;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
// import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
// import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

// import com.ctre.phoenix6.signals.NeutralModeValue;

// public class Elevator extends ServoMotorSubsystem {
// 	public static Elevator mInstance;

// 	public static Elevator getInstance() {
// 		if (mInstance == null) {
// 			mInstance = new Elevator(ElevatorConstants.kElevatorServoConstants);
// 		}
// 		return mInstance;
// 	}

// 	final static double kStrictError = .5;
// 	final static double kMediumError = 2;
// 	final static double kLenientError = 5;

// 	LoggedMechanism2d mech = new LoggedMechanism2d(2,2);
// 	LoggedMechanismRoot2d root = mech.getRoot("Elevator", .85,0);
// 	LoggedMechanismLigament2d low = root.append(new LoggedMechanismLigament2d("low", .4, 85));
// 	public LoggedMechanismLigament2d midBar = low.append(new LoggedMechanismLigament2d("lowBar", .2, -90));
// 	LoggedMechanismLigament2d mid = low.append(new LoggedMechanismLigament2d("mid", .3, 0));
// 	LoggedMechanismLigament2d algaeBar = mid.append(new LoggedMechanismLigament2d("algaeBar", .2, -90));

//     public enum State {
//         L4(1.4, kStrictError),
//         L3(.8, kStrictError),
//         L2(0.2, kStrictError),
//         L1(0.2, kStrictError),
//         A1(0.2, kMediumError),
//         A2(0.8, kMediumError),
//         NET(0.0 ,kMediumError),
//         ZERO(0.0, kLenientError),
// 		PROCESS(0.0, kLenientError),
//         STOW(0.0, kStrictError);

//         double output = 0;
// 		double allowable_error = 20;

//         State(double output, double allowable_error) {
//         this.output = output;
// 		this.allowable_error = allowable_error;
//         }
//     }


// 	public Elevator(final ServoMotorSubsystemConstants constants) {
// 		super(constants);
// 		mMain.setPosition(homeAwareUnitsToRotations(0.0));
// 		enableSoftLimits(false);
// 		setSetpointMotionMagic(State.ZERO.output);
// 	}

// 	public void registerEnabledLoops(ILooper enabledLooper) {
// 		enabledLooper.register(new Loop() {
// 			@Override
// 			public void onStart(double timestamp) {}

// 			@Override
// 			public void onLoop(double timestamp) {
// 		}

// 			@Override
// 			public void onStop(double timestamp) {
// 				setNeutralMode(NeutralModeValue.Brake);
// 			}
// 		});
// 	}

//     @Override
//     public synchronized void readPeriodicInputs() {
//         super.readPeriodicInputs();
//         Logger.processInputs("Elevator", mServoInputs);
//     }

// 	@Override
// 	public synchronized void writePeriodicOutputs() {
// 		super.writePeriodicOutputs();
// 	}

// 	@Override
// 	public void outputTelemetry() {

// 		low.setLength(.4+mServoInputs.position_units);

// 		Logger.recordOutput("Elevator/Mech", mech);
// 		SmartDashboard.putBoolean(mConstants.kName + "/Within Homing Window", atHomingLocation());
// 		super.outputTelemetry();
// 	}

// 	@Override
// 	public void stop() {}

// 	@Override
// 	public boolean checkSystem() {
// 		return false;
// 	}

// 	public Request stowRequest() {
// 		return new Request() {
// 			@Override
// 			public void act() {
// 				setSetpointMotionMagic(State.STOW.output);
// 			}

// 			@Override
// 			public boolean isFinished() {
// 				return Util.epsilonEquals(getPosition(), State.STOW.output, State.STOW.allowable_error);
// 			}
// 		};
// 	}

// 	public Request zeroRequest() {
// 		return new Request() {
// 			@Override
// 			public void act() {
// 				setSetpointMotionMagic(State.ZERO.output);
// 			}
// 			@Override
// 			public boolean isFinished() {
// 				return Util.epsilonEquals(getPosition(), State.ZERO.output, State.ZERO.allowable_error);
// 			}
// 		};
// 	}	

// 	public Request waitForExtensionRequest(double position){
// 		return new Request() {
// 			@Override
// 			public void act() {
// 			}

// 			@Override
// 			public boolean isFinished() {
// 				return mServoInputs.position_units >= position;
// 			}
// 		};
// 	}


// 	public Request stateRequest(State _wantedState) {
// 		return new Request() {
// 			@Override
// 			public void act() {
// 				setSetpointMotionMagic(_wantedState.output);
// 			}
// 			@Override
// 			public boolean isFinished() {
// 				return Util.epsilonEquals(getPosition(), _wantedState.output, _wantedState.allowable_error);
// 			}
// 		};
// 	}

// }