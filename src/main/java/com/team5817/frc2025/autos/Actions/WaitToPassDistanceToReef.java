package com.team5817.frc2025.autos.Actions;

import com.team5817.frc2025.Constants;
import com.team5817.frc2025.field.FieldLayout;
import com.team5817.frc2025.subsystems.Drive.Drive;

public class WaitToPassDistanceToReef implements Action {
    double startingDistance;
    double targetDistance;
    Drive mDrive;

    public WaitToPassDistanceToReef() {
        this(Constants.kDefaultDistanceToReef);
    }

    public WaitToPassDistanceToReef(double distance) {
        this.targetDistance = distance;
        this.mDrive = Drive.getInstance();
    }

    @Override
    public boolean isFinished() {

        double currentDistance = mDrive.getPose().getTranslation().translateBy(FieldLayout.getReefPose().inverse().getTranslation()).norm();
        return Math.signum(startingDistance - targetDistance) != Math.signum(currentDistance - targetDistance);
    }

    @Override
    public void start() {
        startingDistance = mDrive.getPose().getTranslation().translateBy(FieldLayout.getReefPose().inverse().getTranslation()).norm();
    }

    @Override
    public void update() {}

    @Override
    public void done() {}
}
