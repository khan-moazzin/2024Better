package com.team5817.frc2025.controlboard;

import org.littletonrobotics.junction.Logger;

import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.Constants;
import com.team5817.frc2025.field.FieldLayout;
import com.team5817.frc2025.subsystems.LEDs;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Superstructure.GoalState;
import com.team5817.frc2025.subsystems.Drive.Drive;
import com.team5817.frc2025.subsystems.Drive.Drive.DriveControlState;
import com.team5817.frc2025.subsystems.Elevator.Elevator;
import com.team5817.frc2025.subsystems.EndEffector.EndEffectorWrist;
import com.team5817.frc2025.subsystems.Intake.IntakeDeploy;

import edu.wpi.first.wpilibj.Timer;

/**
 * The DriverControls class handles the input from the driver and co-driver controllers
 * and translates them into actions for the robot's subsystems.
 */
public class DriverControls {

	ControlBoard mControlBoard = ControlBoard.getInstance();

	Superstructure s;
	Drive mDrive;

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


		if(driver.getAButton())
			IntakeDeploy.getInstance().stateRequest(IntakeDeploy.State.DEPLOY).act();
		if(driver.getBButton())
			IntakeDeploy.getInstance().stateRequest(IntakeDeploy.State.STOW).act();
		
		
			
	}

	boolean climbAllowed = false;
	boolean autoAlignAllowed = true;
	boolean codriverManual = false;
	CustomXboxController driver = mControlBoard.driver;
	CustomXboxController codriver = mControlBoard.operator;
	/* TWO CONTROLLERS */
	GoalState preparedGoal = GoalState.L4;
	Timer practiceTimer = new Timer();

	boolean wantStow = false;
	/**
	 * Handles the input for the two controller mode.
	 * This mode is used when both driver and co-driver controllers are available.
	 */
	public void twoControllerMode() {

		if(driver.getStartButton())
			mDrive.zeroGyro();
		
		if(!climbAllowed){
			// if(driver.\)sswaaaaaa
			if(driver.leftBumper.isBeingPressed()){
				s.setGoal(GoalState.GROUND_CORAL_INTAKE);
			}
			if(driver.leftTrigger.isBeingPressed()){
				s.setGoal(preparedGoal);
				if(preparedGoal == GoalState.L4){
					s.mDrive.setDriverKinematicLimits(Constants.SwerveConstants.kExtendedKinematicLimits);
				}else
					s.mDrive.setDriverKinematicLimits(Constants.SwerveConstants.kSwerveKinematicLimits);

			}else
				s.mDrive.setDriverKinematicLimits(Constants.SwerveConstants.kSwerveKinematicLimits);

			if(driver.rightTrigger.isBeingPressed()){
				if(autoAlignAllowed&&s.getGoalState().goal.mAlignmentType!=AlignmentType.NONE)
					mDrive.autoAlign(s.getGoalState().goal.mAlignmentType);
				else
					mDrive.autoAlignFinishedOverrride(true);
			}


			if(driver.aButton.isBeingPressed()){
				s.setGoal(s.AlgaeSmartCleanRequest());
			}
				// s.AlgaeSmartCleanRequest();
			if(driver.bButton.isBeingPressed())
				s.setGoal(GoalState.HUMAN_CORAL_INTAKE);
			
			if(driver.xButton.isBeingPressed())
				s.setGoal(GoalState.GROUND_ALGAE_INTAKE);

			if(driver.getBackButton()){
				EndEffectorWrist.getInstance().conformToState(EndEffectorWrist.State.ZERO);
			}

			
			if(driver.releasedAny(driver.leftBumper,driver.bButton,driver.xButton)){
				s.setGoal(GoalState.STOW);
				mDrive.setControlState(DriveControlState.OPEN_LOOP);
			}


			if(driver.releasedAny(driver.leftTrigger,driver.aButton) ){
				wantStow = true;
			}
			if(wantStow&&clearReef()){
				s.setGoal(GoalState.STOW);
				mDrive.setControlState(DriveControlState.OPEN_LOOP);
				wantStow = false;
			}

			if(driver.releasedAny(driver.rightTrigger))
				mDrive.setControlState(DriveControlState.OPEN_LOOP);
		
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

		
		if(driver.leftTrigger.isBeingPressed())
			s.setReadyToScore(driver.rightBumper.isBeingPressed());
		else if(driver.rightBumper.isBeingPressed())
			s.setGoal(GoalState.EXHAUST);

		Logger.recordOutput("Elastic/Codriver Manual", codriverManual);
		Logger.recordOutput("Elastic/Auto Align Allowed", autoAlignAllowed);
		Logger.recordOutput("Elastic/Climb Allowed", climbAllowed);
		Logger.recordOutput("Elastic/PreparedGoal", preparedGoal);

	}

	public boolean clearReef(){
		return mDrive.getPose().getTranslation().translateBy(FieldLayout.getReefPose().inverse().getTranslation()).norm() > 1.4;
	}

	
}
