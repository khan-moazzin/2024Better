package com.team5817.lib.motion;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.geometry.Pose2d;

public class TrajectorySet {
    List<Trajectory> mTrajectorySet = new ArrayList<>();
    boolean mirrored;

    public TrajectorySet(Trajectory... trajectories) {
        mirrored = false;
        for (Trajectory t : trajectories) {
            t.setMirrored(mirrored);
            mTrajectorySet.add(t);
        }
    }

    public TrajectorySet(Boolean mirrored, Trajectory... trajectories) {
        this.mirrored = mirrored;
        for (Trajectory t : trajectories) {
            t.setMirrored(mirrored);
            mTrajectorySet.add(t);
        }

    }


    public Trajectory next() {
        return mTrajectorySet.remove(0);
    }

    public Trajectory first() {
        return mTrajectorySet.get(0);
    }

    public Pose2d initalPose() {

        return mTrajectorySet.get(0).get().getCurrentState().getPose();
    }

}