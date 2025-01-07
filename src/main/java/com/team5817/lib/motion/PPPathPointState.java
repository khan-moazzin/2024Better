package com.team5817.lib.motion;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;

public class PPPathPointState {
    protected final Pose2d mPose;
    protected final Rotation2d mMotionDirection;
    protected final double mHeading_rate;
    protected final double mVelocity;
    protected final double mT;

    public PPPathPointState(){
        mPose = Pose2d.identity();
        mMotionDirection = Rotation2d.identity();
        mVelocity = 0;
        mHeading_rate = 0;
        mT = 0;
    }

    public PPPathPointState(Pose2d pose, Rotation2d motion_direction, double velocity, double t, double heading_rate){
        this.mHeading_rate = heading_rate;
        this.mPose = pose;
        this.mMotionDirection = motion_direction;
        this.mVelocity = velocity;
        this.mT = t;
    }

    public Pose2d getPose(){
        return mPose;
    }   

    public PPPathPointState transformBy(Pose2d transform){
        return new PPPathPointState(mPose.transformBy(transform), mMotionDirection,mVelocity, mT, mHeading_rate);
    }

    public PPPathPointState mirror(){
        return new PPPathPointState(mPose.mirror().getPose(), mMotionDirection.mirror(), mVelocity, mT, mHeading_rate);
    }

    public PPPathPointState mirrorAboutX(double x){
        return new PPPathPointState(mPose.mirrorAboutX(x), mMotionDirection.mirrorAboutX(), mVelocity, mT, mHeading_rate);
    }

    public PPPathPointState mirrorAboutY(double y){
        return new PPPathPointState(mPose.mirrorAboutY(y), mMotionDirection.mirrorAboutY(), mVelocity, mT, mHeading_rate);
    }


    public double getVelocity(){        
        return mVelocity;
    }

   
    public Translation2d getTranslation(){
        return mPose.getTranslation();    
    }   

    public PPPathPointState interpolate(final PPPathPointState other, double x) {
        return new PPPathPointState(
                getPose().interpolate(other.getPose(), x),
                mMotionDirection.interpolate(other.mMotionDirection, x),
                Util.interpolate(getVelocity(), other.getVelocity(), x),
                Util.interpolate(mT, other.t(), x),
                Util.interpolate(getHeadingRate(), other.getHeadingRate(), x)
                );
    }

    public double getHeadingRate(){
        return mHeading_rate;
    }
    public double t(){
        return mT;
    }
    public PPPathPointState rotateBy(Rotation2d rotation){
        return new PPPathPointState(mPose.rotateBy(rotation), Rotation2d.identity(), mVelocity, mT, mHeading_rate); 
    }
    
    public PPPathPointState add(PPPathPointState other){
        return this.transformBy(other.getPose());
    }
    
    public Rotation2d getCourse(){        
        return mMotionDirection;
    }



    
}
