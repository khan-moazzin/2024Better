package com.team5817.lib.motion;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.team254.lib.trajectory.TrajectoryIterator;
import com.team5817.frc2025.autos.TrajectoryLibrary.l;
import com.team5817.frc2025.subsystems.Drive.SwerveConstants;
import com.team5817.lib.Util;

/**
 * Represents a trajectory for the robot to follow.
 */
public class Trajectory {
    PathPlannerPath mPath;
    PathPlannerTrajectory mTrajectory;
    boolean mMirrored;

    /**
     * Constructs a Trajectory object from a given path name.
     *
     * @param path_name The name of the path file.
     */
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

    /**
     * Sets whether the trajectory should be mirrored.
     *
     * @param mirrored True if the trajectory should be mirrored, false otherwise.
     */
    public void setMirrored(boolean mirrored){
        mMirrored = mirrored;
    }

    /**
     * Updates the trajectory based on the current path and robot configuration.
     */
    public void update() {
        mTrajectory = mPath.getIdealTrajectory(SwerveConstants.mRobotConfig).get();
    }

    /**
     * Gets the current trajectory iterator, considering any mirroring or flipping.
     *
     * @return The trajectory iterator.
     */
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