package com.team5817.lib.motion;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class PPPathPointState {
    protected final Pose2d mPose;
    protected final double xVel;
    protected final double yVel;
    protected final double omegaRadiansPerSecond;
    protected final double mT;

    public PPPathPointState() {
        mPose = Pose2d.identity();
        xVel = 0;
        yVel = 0;
        mT = 0;
        omegaRadiansPerSecond = 0;
    }

    public PPPathPointState(Pose2d pose, double xVel, double yVel, double omegathetaradians, double t) {
        this.mPose = pose;
        this.xVel = xVel;
        this.yVel = yVel;
        this.omegaRadiansPerSecond = omegathetaradians;
        this.mT = t;
    }

    public Pose2d getPose() {
        return mPose;
    }

    public PPPathPointState mirrorAboutX(double x) {
        return new PPPathPointState(mPose.mirrorAboutX(x), -xVel, yVel, -omegaRadiansPerSecond, mT);
    }

    public PPPathPointState mirrorAboutY(double y) {
        return new PPPathPointState(mPose.mirrorAboutY(y), xVel, yVel, omegaRadiansPerSecond, mT);
    }

    public double getXVel() {
        return xVel;
    }

    public double getYVel() {
        return yVel;
    }

    public double getThetaVel() {
        return omegaRadiansPerSecond;
    }

    public Translation2d getTranslation() {
        return mPose.getTranslation();
    }

    public double t() {
        return mT;
    }

    public PPPathPointState rotateBy(Rotation2d rotation) {
        return new PPPathPointState(mPose.rotateBy(rotation), xVel, yVel, omegaRadiansPerSecond, mT);
    }

}
