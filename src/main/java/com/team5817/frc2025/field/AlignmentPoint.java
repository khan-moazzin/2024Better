package com.team5817.frc2025.field;

import java.util.List;

import com.team254.lib.geometry.Translation2d;

public class AlignmentPoint {

    private List<AlignmentType> mAllowedAllignments;
    private Translation2d mPose;

    public AlignmentPoint (Translation2d pose, AlignmentType... allowed_allignments){
        this.mPose = pose;
        this.mAllowedAllignments = List.of(allowed_allignments);
    }

    public enum AlignmentType{
        CORAL_SCORE,
        ALGAE_CLEAN,
        ALGAE_SCORE,
        HUMAN,
        NONE
    }

    public Translation2d getTranslation(){
        return mPose;
    }

    public List<AlignmentType> getAllowedAllignments(){
        return mAllowedAllignments;
    }
    
}
