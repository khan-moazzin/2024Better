package com.team5817.lib.motion;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class PPTimeView {

    private PathPlannerTrajectory mTrajectory;
    private double start_t;
    private double end_t;

    public PPTimeView(PathPlannerTrajectory path) {
        this.mTrajectory = path;
        this.start_t = 0;
        this.end_t = path.getTotalTimeSeconds();
    }

    public double first_interpolant() {
        return start_t;
    }

    public double last_interpolant() {
        return end_t;
    }

    public PPPathPointState sample(double t) {

        PathPlannerTrajectoryState state = mTrajectory.sample(t);

        Pose2d pose = new Pose2d(new Translation2d(state.pose.getTranslation()),
                new Rotation2d(state.pose.getRotation()));
        double xVelocity = state.fieldSpeeds.vxMetersPerSecond;
        double yVelocity = state.fieldSpeeds.vyMetersPerSecond;
        double omegaRadiansPerSecond = state.fieldSpeeds.omegaRadiansPerSecond;

        return new PPPathPointState(pose, xVelocity, yVelocity, omegaRadiansPerSecond, t);
    }

    public PathPlannerTrajectory getTrajectory() {
        return mTrajectory;
    }

}
