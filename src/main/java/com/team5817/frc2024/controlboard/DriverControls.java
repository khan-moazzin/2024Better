package com.team5817.frc2024.controlboard;

import com.team5817.frc2024.subsystems.LEDs;
import com.team5817.frc2024.subsystems.Superstructure;
import com.team5817.frc2024.subsystems.Superstructure.GoalState;
import com.team5817.frc2024.subsystems.Drive.Drive;

public class DriverControls {

	ControlBoard mControlBoard = ControlBoard.getInstance();

	Superstructure s = Superstructure.getInstance();
	Drive mDrive;
	LEDs mLEDs = LEDs.getInstance();
	public DriverControls(){
		mDrive = Drive.getInstance();
	}

	/* ONE CONTROLLER */

	public void oneControllerMode() {
			mDrive.overrideHeading(true);
	}

	/* TWO CONTROLLERS */
	GoalState preparedGoal = GoalState.L4;
	public void twoControllerMode() {
		if(mControlBoard.driver.leftBumper.isBeingPressed()){
			s.request(s.GoalRequest(GoalState.GROUND_CORAL_INTAKE));
		}
		if(mControlBoard.driver.getLeftTriggerAxis()>0.2){
			s.request(s.GoalRequest(preparedGoal));
		}
		if(mControlBoard.driver.rightBumper.isBeingPressed())
			mControlBoard.setSwerveScalar(.5);
		else
			mControlBoard.setSwerveScalar(1);
		if(mControlBoard.driver.aButton.isBeingPressed()){
			s.request(s.AlgaeSmartCleanRequest());;
		}
	}

}
