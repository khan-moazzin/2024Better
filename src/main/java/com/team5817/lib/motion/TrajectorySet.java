package com.team5817.lib.motion;

import java.util.List;

import com.team5817.frc2025.autos.TrajectoryLibrary.l;

public class TrajectorySet {
    List<Trajectory> mTrajectorySet;

    public TrajectorySet(Trajectory... trajectories){
        for (Trajectory t:trajectories){
            mTrajectorySet.add(t);
        }
    }

    public Trajectory next(){
        try{
            return mTrajectorySet.remove(0);
        }
        catch(Exception e){
            System.out.println("Trajectory Set out of bounds");
            return l.empty;
        }
    }
}