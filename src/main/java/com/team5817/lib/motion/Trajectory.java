package com.team5817.lib.motion;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team5817.frc2025.Constants.SwerveConstants;
import com.team5817.frc2025.autos.TrajectoryLibrary.l;

public class Trajectory {
    PathPlannerPath mPath;
    PathPlannerTrajectory mTrajectory;
    boolean mUseSpecializied;

    public Trajectory(String path_name, boolean useSpecialized) {
        try {
            mPath = PathPlannerPath.fromPathFile(path_name);
        } catch (Exception e) {
            System.out.println(e);
        }

        mUseSpecializied = useSpecialized;
        mTrajectory = mPath.getIdealTrajectory(SwerveConstants.mRobotConfig).get();

        l.trajectories.add(this);

    }

    public Trajectory(String path_name) {
        try {
            mPath = PathPlannerPath.fromPathFile(path_name);
        } catch (Exception e) {
            System.out.println(e);
        }
        mUseSpecializied = false;
        mTrajectory = mPath.getIdealTrajectory(SwerveConstants.mRobotConfig).get();

        l.trajectories.add(this);
    }

    public void update() {
        mTrajectory = mPath.getIdealTrajectory(SwerveConstants.mRobotConfig).get();
    }

    public boolean useSpecialized() {
        return mUseSpecializied;
    }

    public TrajectoryIterator get() {
        return new TrajectoryIterator(new PPTimeView(mTrajectory), mPath);
    }
}