package com.team5817.lib.motion;

import java.util.List;

public class TrajectorySet {
    List<Trajectory> mTrajectorySet;

    public TrajectorySet(Trajectory... trajectories){
        for (Trajectory t:trajectories){
            mTrajectorySet.add(t);
        }
    }


    public Trajectory next(){
        return mTrajectorySet.remove(0);
    }
    
}
