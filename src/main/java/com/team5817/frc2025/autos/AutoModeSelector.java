package com.team5817.frc2025.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;

import com.team5817.frc2025.autos.Modes.CustomMode;
import com.team5817.frc2025.autos.Modes.DoNothingMode;

/**
 * This class is responsible for selecting the autonomous mode for the robot.
 */
public class AutoModeSelector {

	public enum ScoringLocation{
		_7A,
		_7B,
		_8A,
		_8B,
		_3A,
		_6B
	}

	public enum PickupLocation{
		FAR,
		CLOSE,
	}

	public enum DesiredMode {
		DO_NOTHING,
		THREE_CORAL_MODE,
		CUSTOM_MODE,
	}
	public enum StartingPosition {

		PROCCESSOR_SIDE(DesiredMode.CUSTOM_MODE, DesiredMode.THREE_CORAL_MODE, DesiredMode.DO_NOTHING),
		CENTER_PROCESS(DesiredMode.CUSTOM_MODE, DesiredMode.THREE_CORAL_MODE, DesiredMode.DO_NOTHING),
		CENTER_BLANK(true, DesiredMode.CUSTOM_MODE, DesiredMode.THREE_CORAL_MODE, DesiredMode.DO_NOTHING),
		BLANK_SIDE(true, DesiredMode.CUSTOM_MODE, DesiredMode.THREE_CORAL_MODE, DesiredMode.DO_NOTHING);

		public List<DesiredMode> modes;
		public Boolean mirrored = false;
		private StartingPosition(DesiredMode... modes) {
			this.modes = List.of(modes);
			this.mirrored = false;
		} 
		private StartingPosition(Boolean mirrored, DesiredMode... modes) {
			this.modes = List.of(modes);
			this.mirrored = mirrored;
		}
	}

	private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;
	private StartingPosition mCachedStartingPosition = StartingPosition.PROCCESSOR_SIDE;
	private PickupLocation mCachedPickupLocation = PickupLocation.FAR;
	private ScoringLocation mCachedFirstScore = ScoringLocation._3A;
	private ScoringLocation mCachedSecondScore = ScoringLocation._7A;
	private ScoringLocation mCachedThirdScore = ScoringLocation._7B;
	private int mCachedScoreAmount = 3;


	private Optional<AutoBase> mAutoMode = Optional.empty();

	private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();
	private static SendableChooser<StartingPosition> mStartingPositionSelector = new SendableChooser<>();
	private static SendableChooser<PickupLocation> mPickupLocationSelector = new SendableChooser<>();

	private static SendableChooser<ScoringLocation> mFirstScoreSelector = new SendableChooser<>();
	private static SendableChooser<ScoringLocation> mSecondScoreSelector = new SendableChooser<>();
	private static SendableChooser<ScoringLocation> mThirdScoreSelector = new SendableChooser<>();
	private static SendableChooser<Integer> mScoreAmountSelector = new SendableChooser<>();


	/**
	 * Constructor for AutoModeSelector.
	 * Initializes the SendableChoosers for starting position, pickup location, and scoring locations.
	 */
	public AutoModeSelector() {
		mStartingPositionSelector.setDefaultOption("Proccessor Side", StartingPosition.PROCCESSOR_SIDE);
		mStartingPositionSelector.addOption("Center Processor Side", StartingPosition.CENTER_PROCESS);
		mStartingPositionSelector.addOption("Center Blank Side", StartingPosition.CENTER_BLANK);
		mStartingPositionSelector.addOption("Blank Side", StartingPosition.BLANK_SIDE);

		mPickupLocationSelector.setDefaultOption("Far", PickupLocation.FAR);
		mPickupLocationSelector.addOption("Close", PickupLocation.CLOSE);

		mFirstScoreSelector.setDefaultOption("3A", ScoringLocation._3A);
		mFirstScoreSelector.addOption("8A", ScoringLocation._8A);
		mFirstScoreSelector.addOption("8B", ScoringLocation._8B);
		mSecondScoreSelector.setDefaultOption("7A", ScoringLocation._7A);
		mSecondScoreSelector.addOption("7B", ScoringLocation._7B);
		mSecondScoreSelector.addOption("8A", ScoringLocation._8A);
		mSecondScoreSelector.addOption("8B", ScoringLocation._8B);
		mSecondScoreSelector.addOption("3A", ScoringLocation._3A);
		mSecondScoreSelector.addOption("6B", ScoringLocation._6B);
		mThirdScoreSelector.setDefaultOption("7B", ScoringLocation._7B);
		mThirdScoreSelector.addOption("7A", ScoringLocation._7A);
		mThirdScoreSelector.addOption("8A", ScoringLocation._8A);
		mThirdScoreSelector.addOption("8B", ScoringLocation._8B);
		mThirdScoreSelector.addOption("3A", ScoringLocation._3A);
		mThirdScoreSelector.addOption("6B", ScoringLocation._6B);

		mScoreAmountSelector.setDefaultOption("3", 1);
		mScoreAmountSelector.addOption("2", 2);
		mScoreAmountSelector.addOption("1", 3);
		
	}

	/**
	 * Updates the mode creator based on the selected starting position and desired mode.
	 * Updates the cached values for pickup location and scoring locations.
	 */
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

		mCachedPickupLocation = mPickupLocationSelector.getSelected();
		mCachedFirstScore = mFirstScoreSelector.getSelected();
		mCachedSecondScore = mSecondScoreSelector.getSelected();
		mCachedThirdScore = mThirdScoreSelector.getSelected();
		mCachedScoreAmount = mScoreAmountSelector.getSelected();
		
		SmartDashboard.putData("Starting Position", mStartingPositionSelector);
		SmartDashboard.putData("Auto Mode", mModeChooser);
		SmartDashboard.putData("Pickup Location", mPickupLocationSelector);

		if(desiredMode == DesiredMode.CUSTOM_MODE){
			SmartDashboard.putData("First Score Selector", mFirstScoreSelector);
			SmartDashboard.putData("Second Score Selector", mSecondScoreSelector);
			SmartDashboard.putData("Third Score Selector", mThirdScoreSelector);
			SmartDashboard.putData("Score Amount", mScoreAmountSelector);

		}
		
}

	/**
	 * Returns the AutoBase instance for theputp given desired mode.
	 *
	 * @param mode The desired autonomous mode.
	 * @return An Optional containing the AutoBase instance if a valid mode is found, otherwise an empty Optional.
	 */
	private Optional<AutoBase> getAutoModeForParams(DesiredMode mode) {
		switch (mode) {
			case DO_NOTHING:
				return Optional.of(new DoNothingMode());

			case CUSTOM_MODE:
				return Optional.of(new CustomMode(mCachedStartingPosition, mCachedPickupLocation, mCachedFirstScore, mCachedSecondScore, mCachedThirdScore));

		default:
				System.out.println("ERROR: unexpected auto mode: " + mode);
				break;
		}

		System.err.println("No valid auto mode found for  " + mode);
		return Optional.empty();
	}

	/**
	 * Returns the SendableChooser for selecting the desired mode.
	 *
	 * @return The SendableChooser for desired mode.
	 */
	public static SendableChooser<DesiredMode> getModeChooser() {
		return mModeChooser;
	}

	/**
	 * Returns the cached desired autonomous mode.
	 *
	 * @return The cached desired mode.
	 */
	public DesiredMode getDesiredAutomode() {
		return mCachedDesiredMode;
	}

	/**
	 * Resets the AutoModeSelector by clearing the cached auto mode and desired mode.
	 */
	public void reset() {
		mAutoMode = Optional.empty();
		mCachedDesiredMode = null;
	}

	/**
	 * Outputs the selected autonomous mode and starting position to the SmartDashboard.
	 */
	public void outputToSmartDashboard() {
		SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
		SmartDashboard.putString("Starting Position Selected", mCachedStartingPosition.name());
	}

	/**
	 * Returns the currently selected AutoBase instance.
	 *
	 * @return An Optional containing the AutoBase instance if present, otherwise an empty Optional.
	 */
	public Optional<AutoBase> getAutoMode() {

		if (!mAutoMode.isPresent()) {
			return Optional.empty();
		}

		return mAutoMode;
	}
}
