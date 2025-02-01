package com.team5817.frc2025.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

import com.team5817.frc2025.autos.Modes.DoNothingMode;
import com.team5817.frc2025.autos.Modes.ThreeCoralMode;

public class AutoModeSelector {
	public enum DesiredMode {
		DO_NOTHING,
		THREE_CORAL_MODE
	}


	private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;

	private Optional<AutoBase> mAutoMode = Optional.empty();

	private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();

	public AutoModeSelector() {
		mModeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);
		mModeChooser.addOption("Three Coral Mode", DesiredMode.THREE_CORAL_MODE);

		SmartDashboard.putData("Auto Mode", mModeChooser);

	}

	public void updateModeCreator() {
		DesiredMode desiredMode = mModeChooser.getSelected();
		
		if (desiredMode == null) {
			desiredMode = DesiredMode.DO_NOTHING;
		}
		mAutoMode = getAutoModeForParams(desiredMode);
}

	private Optional<AutoBase> getAutoModeForParams(DesiredMode mode) {
		switch (mode) {
			case DO_NOTHING:
				return Optional.of(new DoNothingMode());


			case THREE_CORAL_MODE:
				return Optional.of(new ThreeCoralMode());
			// break;

		default:
				System.out.println("ERROR: unexpected auto mode: " + mode);
				break;
		}

		System.err.println("No valid auto mode found for  " + mode);
		return Optional.empty();
	}

	public static SendableChooser<DesiredMode> getModeChooser() {
		return mModeChooser;
	}

	public DesiredMode getDesiredAutomode() {
		return mCachedDesiredMode;
	}

	public void reset() {
		mAutoMode = Optional.empty();
		mCachedDesiredMode = null;
	}

	public void outputToSmartDashboard() {
		SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
	}

	public Optional<AutoBase> getAutoMode() {

		if (!mAutoMode.isPresent()) {
			return Optional.empty();
		}

		return mAutoMode;
	}
}
