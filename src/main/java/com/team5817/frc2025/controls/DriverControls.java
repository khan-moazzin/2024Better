package com.team5817.frc2025.controls;

import org.littletonrobotics.junction.Logger;


import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Superstructure.GoalState;
import com.team5817.frc2025.subsystems.Superstructure.Mode;

import com.team5817.frc2025.subsystems.Drive.Drive;




public class DriverControls {
    private final CustomXboxController driver;
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Drive drive = Drive.getInstance();


    public DriverControls() {
        this.driver = CustomXboxController.getInstance();
		
    }

    public void update() {
		superstructure.setMode(Mode.SHOOTING);

		// Reset Gyro
        if (driver.getStartButtonPressed()) {
            drive.zeroGyro();
        }

        // Drive control
        double xSpeed = -driver.getLeftY() * 0.8;
        double ySpeed = driver.getLeftX() * 0.8;
        double rot = driver.getRightX();
        drive.sendInput(xSpeed, ySpeed, rot);


        // Pivot offset 
        if (driver.dpadUp.wasActivated()) {
            superstructure.offsetPivot(1);
        } else if (driver.dpadDown.wasActivated()) {
            superstructure.offsetPivot(-1);
        }
		

		// GoalState transitions 
		if (driver.leftTrigger.getValue() > 0.2) {
			superstructure.setGoal(GoalState.OUTTAKE);
		} else if (driver.rightTrigger.getValue() > 0.2) {
			superstructure.setGoal(GoalState.INTAKE);
		} else if (driver.rightBumper.isActive()) {
			if (driver.leftBumper.isActive()) {
				superstructure.setGoal(GoalState.SHOOTING);
			} else {
				superstructure.setGoal(GoalState.IDLE);
			}
		} 



        // No Rumble as we dont have an active beambreak available to implement in the indexer
        //driver.setRumble(Intake.getInstance().hasPiece());

        
		// Log current GoalState
        Logger.recordOutput("DriverControls/GoalState", superstructure.getGoalState());
    }
}
