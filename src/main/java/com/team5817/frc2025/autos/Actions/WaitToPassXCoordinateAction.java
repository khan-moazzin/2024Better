package com.team5817.frc2025.autos.Actions;


import com.team5817.frc2025.field.FieldLayout;
import com.team5817.frc2025.subsystems.Drive.Drive;

import edu.wpi.first.wpilibj.DriverStation;

public class WaitToPassXCoordinateAction implements Action {
	double startingXCoordinate;
	double targetXCoordinate;
	Drive mDrive;

	public WaitToPassXCoordinateAction(double x) {
		if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
			targetXCoordinate = FieldLayout.kFieldLength - x;
		} else {
			targetXCoordinate = x;
		}
		mDrive = Drive.getInstance();
	}

	@Override
	public boolean isFinished() {
		return Math.signum(startingXCoordinate - targetXCoordinate)
				!= Math.signum(mDrive.getPose().getTranslation().x() - targetXCoordinate);
	}

	@Override
	public void start() {
		startingXCoordinate = mDrive.getPose().getTranslation().x();
	}

	@Override
	public void update() {}

	@Override
	public void done() {}
}