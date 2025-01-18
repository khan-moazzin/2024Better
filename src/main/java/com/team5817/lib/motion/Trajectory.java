package com.team5817.lib.motion;

import java.util.Optional;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team5817.frc2025.Constants.SwerveConstants;

public class Trajectory {
    Optional<PathPlannerPath> mPath;
    PathPlannerTrajectory mTrajectory;

    
    public Trajectory(String path_name){

        try {
            mPath = Optional.of(PathPlannerPath.fromPathFile(path_name));
        } catch (Exception e) {
            mPath = Optional.empty();
        }

        if(mPath.isEmpty()){
            return;
        }

        mTrajectory = mPath.get().getIdealTrajectory(SwerveConstants.mRobotConfig).get();

    }
    
    public TrajectoryIterator get(){
        return new TrajectoryIterator(new PPTimeView(mTrajectory));
    }
}