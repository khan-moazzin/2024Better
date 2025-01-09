package com.team5817.frc2024.subsystems.Elevator;

import com.team5817.lib.Util;
import com.team5817.lib.drivers.ServoMotorSubsystem;
import com.team5817.lib.requests.Request;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2024.Constants.ElevatorConstants;
import com.team5817.frc2024.loops.ILooper;
import com.team5817.frc2024.loops.Loop;

public class Elevator extends ServoMotorSubsystem {
	public static Elevator mInstance;

	public static Elevator getInstance() {
		if (mInstance == null) {
			mInstance = new Elevator(ElevatorConstants.kElevatorServoConstants);
		}
		return mInstance;
	}

    public enum State {
        L4(0.0),
        L3(0.0),
        L2(0.0),
        L1(0.0),
        A1(0.0),
        A2(0.0),
        NET(0.0),
        ZERO(0.0),
        STOW(0.0);

        double output = 0;

        State(double output) {
        this.output = output;
        }
    }


	public Elevator(final ServoMotorSubsystemConstants constants) {
		super(constants);
		mMain.setPosition(homeAwareUnitsToRotations(120.0));
		enableSoftLimits(false);
		setSetpointMotionMagic(State.ZERO.output);
	}

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
		}

			@Override
			public void onStop(double timestamp) {
				setNeutralMode(NeutralModeValue.Brake);
			}
		});
	}

    @Override
    public synchronized void readPeriodicInputs() {
        super.readPeriodicInputs();
        Logger.processInputs("Elevator", mServoInputs);
    }

	@Override
	public synchronized void writePeriodicOutputs() {
		super.writePeriodicOutputs();
	}

	@Override
	public void outputTelemetry() {

		SmartDashboard.putBoolean(mConstants.kName + "/Within Homing Window", atHomingLocation());
		super.outputTelemetry();
	}

	@Override
	public void stop() {}

	@Override
	public boolean checkSystem() {
		return false;
	}

	/**
	 * @return Request to move intake to deploy angle to pick up notes off the ground.
	 */
	public Request l1Request() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.L1.output);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.L1.output, 20.0);
			}
		};
	}

	public Request l2Request() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.L2.output);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.L2.output, 20.0);
			}
		};
	}

	public Request l3Request() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.L1.output);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.L3.output, 20.0);
			}
		};
	}

	public Request l4Request() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.L4.output);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.L1.output, 20.0);
			}
		};
	}

	public Request stowRequest() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.STOW.output);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.STOW.output, 20.0);
			}
		};
	}

	public Request zeroRequest() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.ZERO.output);
			}
			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.ZERO.output, 20.0);
			}
		};
	}

	public Request netRequest() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.NET.output);
			}
			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.NET.output, 20.0);
			}
		};
	}	

	public Request a1Request() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.A1.output);
			}
			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.A1.output, 20.0);
			}
		};
	}

	public Request a2Request() {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(State.A2.output);
			}
			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), State.A2.output, 20.0);
			}
		};
	}

	public Request stateRequest(State _wantedState) {
		return new Request() {
			@Override
			public void act() {
				setSetpointMotionMagic(_wantedState.output);
			}
			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(getPosition(), _wantedState.output, 20.0);
			}
		};
	}

}