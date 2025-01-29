package com.team5817.lib.motion;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.geometry.Pose2d;

public class TrajectorySet {
    List<Trajectory> mTrajectorySet = new ArrayList<>();
    Pose2d mInitalPose;
    public TrajectorySet(Trajectory... trajectories){
        for (Trajectory t:trajectories){
            mTrajectorySet.add(t);
        }
        mInitalPose = mTrajectorySet.get(0).get().getCurrentState().getPose();
    }


    public Trajectory next(){
        return mTrajectorySet.remove(0);
    }

    public Trajectory first(){
        return mTrajectorySet.get(0);
    }

    public Pose2d initalPose(){
        return mInitalPose;
    }


    
}