package com.team5817.frc2025.controlboard;

import org.littletonrobotics.junction.Logger;

import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;
import com.team5817.frc2025.Constants;
import com.team5817.frc2025.field.FieldLayout;
import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Superstructure.GoalState;
import com.team5817.lib.Util;
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
			IntakeDeploy.getInstance().stateRequest(IntakeDeploy.State.GROUND).act();
		if(driver.getBButton())
			IntakeDeploy.getInstance().stateRequest(IntakeDeploy.State.STOW).act();
		
		
			
	}
	boolean swap = false;
	boolean climbAllowed = false;
	boolean autoAlignAllowed = true;
	boolean codriverManual = false;
	CustomXboxController driver = mControlBoard.driver;
	CustomXboxController codriver = mControlBoard.operator;
	/* TWO CONTROLLERS */
	GoalState preparedGoal = GoalState.L4;
	GoalState preparedAlgae = GoalState.A1;
	double lastTime = 0;

	boolean wantStow = false;
	/**
	 * Handles the input for the two controller mode.
	 * This mode is used when both driver and co-driver controllers are available.
	 */
	public void twoControllerMode() {

		if(driver.getStartButton())
			mDrive.zeroGyro(Util.isRed().get()?0:180);
		if(driver.getBackButton())
			mDrive.zeroGyro(Util.isRed().get()?180:0);


		
		if(!climbAllowed){
			// if(driver.\)sswaaaaaa
			if(driver.leftBumper.isBeingPressed()){
				s.setGoal(GoalState.GROUND_CORAL_INTAKE);
			}
			if(driver.leftTrigger.isBeingPressed()){
				s.setGoal(preparedGoal);
				s.mDrive.setDriverKinematicLimits(Constants.SwerveConstants.kSwerveKinematicLimits);

			}else
				s.mDrive.setDriverKinematicLimits(Constants.SwerveConstants.kSwerveKinematicLimits);

			if(driver.rightTrigger.isBeingPressed()){
				if(autoAlignAllowed&&s.getGoalState().goal.mAlignmentType!=AlignmentType.NONE)
					mDrive.autoAlign(s.getGoalState().goal.mAlignmentType);
					
				else
					mDrive.autoAlignFinishedOverrride(true);
			}


			if(driver.getAButtonPressed()){
				s.setGoal(preparedAlgae);
			}
			if(codriver.getRightBumperButtonPressed())
				swap = !swap;
			if(driver.bButton.isBeingPressed())
				s.setGoal(GoalState.HUMAN_CORAL_INTAKE);
			
			// if(driver.xButton.isBeingPressed())
			// 	s.setGoal(GoalState.GROUND_ALGAE_INTAKE);

			if(driver.getXButtonPressed()){
				s.mEndEffectorWrist.conformToState(EndEffectorWrist.State.ZERO);
				s.mIntakeDeploy.conformToState(IntakeDeploy.State.ZERO);
				s.mElevator.stateRequest(Elevator.State.ZERO).act();
			}

			
			if(driver.releasedAny(driver.leftBumper,driver.bButton,driver.aButton)){
				
				s.setGoal(GoalState.STOW);
				mDrive.setControlState(DriveControlState.OPEN_LOOP);
			}

			if(driver.releasedAny(driver.leftTrigger) ){
				wantStow = true;
			}
			if(wantStow&&clearReef()){
				s.setGoal(GoalState.STOW);
				
				mDrive.setControlState(DriveControlState.OPEN_LOOP);
				wantStow = false;
			}

			if(driver.releasedAny(driver.rightTrigger))
				mDrive.setControlState(DriveControlState.OPEN_LOOP);
			
			if(driver.leftTrigger.isBeingPressed())
				s.setReadyToScore(driver.rightBumper.isBeingPressed());
			// else if(driver.xButton.isBeingPressed()&&driver.rightBumper.isBeingPressed()){
			// 	s.setGoal(GoalState.GROUND_ALGAE_SHOOT);
			else if(driver.rightBumper.isBeingPressed())
				s.setGoal(GoalState.EXHAUST);
		}else {
			if (driver.aButton.wasActivated())
				s.setGoal(GoalState.CLIMB_PREPARE);
			if(driver.aButton.wasReleased())
				s.setGoal(GoalState.CLIMB_PULL);	
			}


		if(codriver.getRightTriggerAxis()==1)
			s.setGoal(GoalState.PREINTAKE);
		if(codriver.getLeftTriggerAxis()==1)
			s.setGoal(GoalState.CLEAR);
		if(codriver.getStartButtonPressed())
			s.toggleAllowPoseComp();
		if(s.getGoalState()==GoalState.CLEAR&&codriver.getLeftTriggerAxis()!=1)
			s.setGoal(GoalState.STOW);
		if(codriver.yButton.shortReleased())
			preparedGoal = GoalState.L4;
		if(codriver.yButton.longPressed())
			preparedAlgae = GoalState.A2;
		if(codriver.bButton.isBeingPressed())
			preparedGoal = GoalState.L3;
		if(codriver.aButton.shortReleased())
			preparedGoal = GoalState.L2;
		
		if(codriver.aButton.longPressed())
			preparedAlgae = GoalState.A1;
		if(codriver.xButton.isBeingPressed())
			preparedGoal = GoalState.L1;
		if(codriver.POV0.isBeingPressed()){
			preparedGoal = GoalState.NET;
		}
		if(codriver.POV180.isBeingPressed()){
			preparedGoal = GoalState.PROCESS;
		}
		
		if(codriver.getBackButtonPressed()){
			codriverManual = !codriverManual;
		}
		if(codriverManual){
			if(lastTime==0)
				lastTime = Timer.getFPGATimestamp();
			double dt = Timer.getTimestamp()-lastTime;
			s.mElevator.changeManualOffset(-codriver.getLeftY()*dt*0.0254);
			s.mEndEffectorWrist.changeManualOffset(-codriver.getRightY()*dt*50);
			if(codriver.getLeftBumperButtonPressed())
				s.mElevator.setManualOffset(0);
			if(codriver.getRightBumperButtonPressed())
				s.mEndEffectorWrist.setManualOffset(0);
		lastTime = Timer.getTimestamp();
		}
		if(codriver.POV270.hasBeenPressed)
			s.mElevator.home();
		if(codriver.POV90.hasBeenPressed)
			s.mEndEffectorWrist.home();

		if(s.mEndEffectorRollers.gotPiece())
			driver.rumble(6, .5);
		

		Logger.recordOutput("Elastic/Codriver Manual", codriverManual);
		Logger.recordOutput("Elastic/Auto Align Allowed", autoAlignAllowed);
		Logger.recordOutput("Elastic/Climb Allowed", climbAllowed);
		Logger.recordOutput("Elastic/PreparedGoal", preparedGoal);
	}

	public boolean clearReef(){
		return mDrive.getPose().getTranslation().translateBy(FieldLayout.getReefPose().inverse().getTranslation()).norm() > 1.4;
	}
	public void testMode(){
		if(driver.getAButton())
			s.mDrive.snapHeading(driver.getPOVDirection());
	}

	
}
