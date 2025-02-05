package com.team5817.frc2025.controlboard;

import org.littletonrobotics.junction.Logger;

import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.subsystems.LEDs;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Superstructure.GoalState;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Drive.Drive.DriveControlState;

import edu.wpi.first.wpilibj.Timer;

/**
 * The DriverControls class handles the input from the driver and co-driver controllers
 * and translates them into actions for the robot's subsystems.
 */
public class DriverControls {

	ControlBoard mControlBoard = ControlBoard.getInstance();

	Superstructure s;
	Drive mDrive;
	LEDs mLEDs = LEDs.getInstance();

	/**
	 * Constructor for the DriverControls class.
	 * Initializes the Drive and Superstructure instances and sets the initial goal state.
	 */
	public DriverControls(){
		mDrive = Drive.getInstance();
		s = Superstructure.getInstance();
		s.setGoal(GoalState.STOW);
	}

	/* ONE CONTROLLER */

	/**
	 * Handles the input for the one controller mode.
	 * This mode is used when only one controller is available for the driver.
	 */
	public void oneControllerMode() {
			// mDrive.overrideHeading(true);
		if(driver.getStartButton())
			mDrive.zeroGyro();
		
			
	}

	boolean climbAllowed = false;
	boolean autoAlignAllowed = true;
	boolean codriverManual = false;
	CustomXboxController driver = mControlBoard.driver;
	CustomXboxController codriver = mControlBoard.operator;
	/* TWO CONTROLLERS */
	GoalState preparedGoal = GoalState.L4;
	Timer practiceTimer = new Timer();

	/**
	 * Handles the input for the two controller mode.
	 * This mode is used when both driver and co-driver controllers are available.
	 */
	public void twoControllerMode() {
		if(driver.getBButton())
			practiceTimer.start();
		if(driver.yButton.wasActivated()){
			Logger.recordOutput("Timer: ", practiceTimer.get());
			practiceTimer.stop();
			practiceTimer.reset();}

		if(driver.getStartButton())
			mDrive.zeroGyro();
		
		if(!climbAllowed){
			// if(driver.\)sswaaaaaa
			if(driver.leftBumper.isBeingPressed()){
				s.setGoal(GoalState.GROUND_CORAL_INTAKE);
			}
			if(driver.leftTrigger.isBeingPressed()){
				if(autoAlignAllowed&&preparedGoal.goal.mAlignmentType!=AlignmentType.NONE)
					mDrive.autoAlign(preparedGoal.goal.mAlignmentType);
				else
					mDrive.autoAlignFinishedOverrride(true);
				s.setGoal(preparedGoal);
			}
			if(driver.rightBumper.isBeingPressed())
				mControlBoard.setSwerveScalar(.5);
			else
				mControlBoard.setSwerveScalar(1);
			if(driver.aButton.isBeingPressed()){
				s.setGoal(s.AlgaeSmartCleanRequest());
				mDrive.autoAlign(AlignmentType.ALGAE_CLEAN);
			}
				// s.AlgaeSmartCleanRequest();
			if(driver.bButton.isBeingPressed())
				s.setGoal(GoalState.HUMAN_CORAL_INTAKE);
			
			if(driver.xButton.isBeingPressed())
				s.setGoal(GoalState.GROUND_ALGAE_INTAKE);

			
			if(driver.releasedAny(driver.leftBumper,driver.leftTrigger,driver.aButton)){
				s.setGoal(GoalState.STOW);
				mDrive.setControlState(DriveControlState.OPEN_LOOP);
			}

		
		}else {
			if (driver.aButton.wasActivated())
				s.setGoal(GoalState.CLIMB_PREPARE);
			if(driver.aButton.wasReleased())
				s.setGoal(GoalState.CLIMB_PULL);	
			}



		if(codriver.getLeftTriggerAxis()==1)
			s.setGoal(GoalState.STOW);

		if(codriver.yButton.isBeingPressed())
			preparedGoal = GoalState.L4;
		if(codriver.bButton.isBeingPressed())
			preparedGoal = GoalState.L3;
		if(codriver.aButton.isBeingPressed())
			preparedGoal = GoalState.L2;
		if(codriver.xButton.isBeingPressed())
			preparedGoal = GoalState.L1;
		if(codriver.POV0.isBeingPressed())
			preparedGoal = GoalState.NET;
		if(codriver.POV180.isBeingPressed())
			preparedGoal = GoalState.PROCESS;
		
		if(codriverManual){
			//TODO
		}
		if(codriver.getRightTriggerAxis()==1)
			climbAllowed = true;	
		if(codriver.leftCenterClick.isBeingPressed())
			climbAllowed = false;
		
		boolean input = codriver.leftBumper.wasActivated();
		autoAlignAllowed = input?!autoAlignAllowed:autoAlignAllowed;

		Logger.recordOutput("Elastic/Codriver Manual", codriverManual);
		Logger.recordOutput("Elastic/Auto Align Allowed", autoAlignAllowed);
		Logger.recordOutput("Elastic/Climb Allowed", climbAllowed);
		Logger.recordOutput("Elastic/PreparedGoal", preparedGoal);

	}


	
}
