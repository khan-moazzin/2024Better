package com.team5817.frc2025.autos;

import edu.wpi.first.util.MsvcRuntimeException;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.security.PrivateKey;
import java.util.List;
import java.util.Optional;

import com.team5817.frc2025.autos.Modes.DoNothingMode;
import com.team5817.frc2025.autos.Modes.ThreeCoralMode;

public class AutoModeSelector {
	public enum DesiredMode {
		DO_NOTHING,
		THREE_CORAL_MODE;
	}
	public enum StartingPosition {
		PROCCESSOR_SIDE(DesiredMode.THREE_CORAL_MODE,DesiredMode.DO_NOTHING),
		CENTER(DesiredMode.DO_NOTHING),
		BLANK_SIDE(true,DesiredMode.THREE_CORAL_MODE,DesiredMode.DO_NOTHING);
		List<DesiredMode> modes;
		Boolean mirrored = false;
		private StartingPosition(DesiredMode... modes) {
			this.modes = List.of(modes);
			this.mirrored = false;
		} 
		private StartingPosition(Boolean mirrored,DesiredMode... modes) {
			this.modes = List.of(modes);
			this.mirrored = mirrored;
		}
	}

	private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;
	private StartingPosition mCachedStartingPosition = StartingPosition.PROCCESSOR_SIDE;


	private Optional<AutoBase> mAutoMode = Optional.empty();

	private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();
	private static SendableChooser<StartingPosition> mStartingPositionSelector = new SendableChooser<>();

	public AutoModeSelector() {
		mStartingPositionSelector.setDefaultOption("Proccessor Side", StartingPosition.PROCCESSOR_SIDE);
		mStartingPositionSelector.addOption("Center", StartingPosition.CENTER);
		mStartingPositionSelector.addOption("Blank Side", StartingPosition.BLANK_SIDE);
	}
	public void updateModeCreator() {
		if(mCachedStartingPosition!=mStartingPositionSelector.getSelected()&&mStartingPositionSelector.getSelected()!=null){
			mModeChooser = new SendableChooser<>();
			mStartingPositionSelector.getSelected().modes.forEach(m -> mModeChooser.addOption(m.name(), m));
			mCachedStartingPosition = mStartingPositionSelector.getSelected();
		}
		
		DesiredMode desiredMode = mModeChooser.getSelected();
		
		if (desiredMode == null) {
			desiredMode = DesiredMode.DO_NOTHING;
		}
		mAutoMode = getAutoModeForParams(desiredMode);

		SmartDashboard.putData("Starting Position", mStartingPositionSelector);
		SmartDashboard.putData("Auto Mode", mModeChooser);
		
}

	private Optional<AutoBase> getAutoModeForParams(DesiredMode mode) {
		switch (mode) {
			case DO_NOTHING:
				return Optional.of(new DoNothingMode());


			case THREE_CORAL_MODE:
				return Optional.of(new ThreeCoralMode(mCachedStartingPosition.mirrored));
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
		SmartDashboard.putString("Starting Position Selected", mCachedStartingPosition.name());
	}

	public Optional<AutoBase> getAutoMode() {

		if (!mAutoMode.isPresent()) {
			return Optional.empty();
		}

		return mAutoMode;
	}
}
