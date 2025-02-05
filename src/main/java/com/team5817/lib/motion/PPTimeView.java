package com.team5817.lib.motion;

import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

/**
 * A class that provides a time-based view of a PathPlannerTrajectory.
 */
public class PPTimeView {

    private PathPlannerTrajectory mTrajectory;
    private double start_t;
    private double end_t;

    /**
     * Constructs a PPTimeView with the given PathPlannerTrajectory.
     *
     * @param path The PathPlannerTrajectory to view.
     */
    public PPTimeView(PathPlannerTrajectory path) {
        this.mTrajectory = path;
        this.start_t = 0;
        this.end_t = path.getTotalTimeSeconds();
    }

    /**
     * Returns the start time of the trajectory.
     *
     * @return The start time in seconds.
     */
    public double first_interpolant() {
        return start_t;
    }

    /**
     * Returns the end time of the trajectory.
     *
     * @return The end time in seconds.
     */
    public double last_interpolant() {
        return end_t;
    }

    /**
     * Samples the trajectory at the given time.
     *
     * @param t The time at which to sample the trajectory.
     * @return The state of the trajectory at the given time.
     */
    public PPPathPointState sample(double t) {

        PathPlannerTrajectoryState state = mTrajectory.sample(t);
        
        Pose2d pose;
        double xVelocity;
        double yVelocity;
        double omegaRadiansPerSecond;

        
            pose = new Pose2d(new Translation2d(state.pose.getTranslation()),
                    new Rotation2d(state.pose.getRotation()));
            xVelocity = state.fieldSpeeds.vxMetersPerSecond;
            yVelocity = state.fieldSpeeds.vyMetersPerSecond;
            omegaRadiansPerSecond = state.fieldSpeeds.omegaRadiansPerSecond;
       

        return new PPPathPointState(pose, xVelocity, yVelocity, omegaRadiansPerSecond, t);
    }

    /**
     * Returns the PathPlannerTrajectory associated with this view.
     *
     * @return The PathPlannerTrajectory.
     */
    public PathPlannerTrajectory getTrajectory() {
        return mTrajectory;
    }

}
