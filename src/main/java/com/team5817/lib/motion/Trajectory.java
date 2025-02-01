package com.team5817.lib.motion;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team5817.frc2025.Constants.SwerveConstants;
import com.team5817.frc2025.autos.TrajectoryLibrary.l;
import com.team5817.lib.Util;

public class Trajectory {
    PathPlannerPath mPath;
    PathPlannerTrajectory mTrajectory;
    boolean mMirrored;


    public Trajectory(String path_name) {
        try {
            mPath = PathPlannerPath.fromPathFile(path_name);
        } catch (Exception e) {
            System.out.println(e);
        }
        mMirrored = false;
        mTrajectory = mPath.getIdealTrajectory(SwerveConstants.mRobotConfig).get();

        l.trajectories.put(path_name, this);
    }

    public void setMirrored(boolean mirrored){
        mMirrored = mirrored;
    }

    public void update() {
        mTrajectory = mPath.getIdealTrajectory(SwerveConstants.mRobotConfig).get();
    }

    public TrajectoryIterator get() {
        PathPlannerPath  flippedPath = mPath;

        if(Util.isRed().get()){
            flippedPath = mPath.flipPath();
        }
        if(Util.isRed().get() && mMirrored){
            flippedPath = mPath.flipPath().mirrorPath();
        }
        else if(mMirrored){
            flippedPath = mPath.mirrorPath();
        }
        
 
       mTrajectory = flippedPath.getIdealTrajectory(SwerveConstants.mRobotConfig).get();
        return new TrajectoryIterator(new PPTimeView(mTrajectory), flippedPath);
    }
}