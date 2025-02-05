package com.team5817.lib.motion;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

/**
 * Represents the state of a point on a path, including its pose, velocities, and time.
 */
public class PPPathPointState {
    protected final Pose2d mPose;
    protected final double xVel;
    protected final double yVel;
    protected final double omegaRadiansPerSecond;
    protected final double mT;

    /**
     * Default constructor initializes the state with zero velocities and identity pose.
     */
    public PPPathPointState() {
        mPose = Pose2d.identity();
        xVel = 0;
        yVel = 0;
        mT = 0;
        omegaRadiansPerSecond = 0;
    }

    /**
     * Constructs a PPPathPointState with the given pose, velocities, and time.
     *
     * @param pose the pose of the point
     * @param xVel the velocity in the x direction
     * @param yVel the velocity in the y direction
     * @param omegathetaradians the angular velocity in radians per second
     * @param t the time
     */
    public PPPathPointState(Pose2d pose, double xVel, double yVel, double omegathetaradians, double t) {
        this.mPose = pose;
        this.xVel = xVel;
        this.yVel = yVel;
        this.omegaRadiansPerSecond = omegathetaradians;
        this.mT = t;
    }

    /**
     * Returns the pose of the point.
     *
     * @return the pose
     */
    public Pose2d getPose() {
        return mPose;
    }

    /**
     * Mirrors the state about the given x coordinate.
     *
     * @param x the x coordinate to mirror about
     * @return the mirrored state
     */
    public PPPathPointState mirrorAboutX(double x) {
        return new PPPathPointState(mPose.mirrorAboutX(x), -xVel, yVel, -omegaRadiansPerSecond, mT);
    }

    /**
     * Mirrors the state about the given y coordinate.
     *
     * @param y the y coordinate to mirror about
     * @return the mirrored state
     */
    public PPPathPointState mirrorAboutY(double y) {
        return new PPPathPointState(mPose.mirrorAboutY(y), xVel, yVel, omegaRadiansPerSecond, mT);
    }

    /**
     * Returns the velocity in the x direction.
     *
     * @return the x velocity
     */
    public double getXVel() {
        return xVel;
    }

    /**
     * Returns the velocity in the y direction.
     *
     * @return the y velocity
     */
    public double getYVel() {
        return yVel;
    }

    /**
     * Returns the angular velocity in radians per second.
     *
     * @return the angular velocity
     */
    public double getThetaVel() {
        return omegaRadiansPerSecond;
    }

    /**
     * Returns the translation component of the pose.
     *
     * @return the translation
     */
    public Translation2d getTranslation() {
        return mPose.getTranslation();
    }

    /**
     * Returns the time associated with this state.
     *
     * @return the time
     */
    public double t() {
        return mT;
    }

    /**
     * Rotates the state by the given rotation.
     *
     * @param rotation the rotation to apply
     * @return the rotated state
     */
    public PPPathPointState rotateBy(Rotation2d rotation) {
        return new PPPathPointState(mPose.rotateBy(rotation), xVel, yVel, omegaRadiansPerSecond, mT);
    }

}
