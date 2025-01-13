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

	public void twoControllerMode() {
		
        
      if(mControlBoard.driver.getAButton()){
        mDrive.autoAlign();
      }
      if(mControlBoard.driver.getBButton()){
        s.request(s.GoalRequest(GoalState.L2));
		s.addRequestToQueue(s.GoalRequest(GoalState.STOW));
      }
	  if(mControlBoard.driver.getXButton()){
		s.request(s.GoalRequest(GoalState.GROUND_CORAL_INTAKE));
		s.addRequestToQueue(s.GoalRequest(GoalState.STOW));
	  }
	  
	}

}
