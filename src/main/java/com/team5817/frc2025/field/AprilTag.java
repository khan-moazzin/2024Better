package com.team5817.frc2025.field;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

public class AprilTag {

    private int id;
    private Pose2d fieldToTag;
    private boolean isScoring;

    private Translation2d tagToLeftAlign;

    private Translation2d tagToRightAlign;

    private Translation2d tagToCenterAlign;

    public AprilTag(int id, boolean isScoring, Translation2d tagToCenterAlign, Translation2d tagToLeftAlign, Translation2d tagToRightAlign) {
        this.id = id;
        this.isScoring = isScoring;
        this.tagToCenterAlign = tagToCenterAlign;
        this.tagToLeftAlign = tagToLeftAlign;
        this.tagToRightAlign = tagToRightAlign;
        this.fieldToTag = new Pose2d(FieldLayout.kTagMap.getTagPose(id).get().toPose2d());
        if(isScoring){
            this.fieldToTag = fieldToTag.withRotation(fieldToTag.getRotation().flip());
        }
        
    }

    public int getId() {
        return id;
    }



    public Pose2d getFieldToTag() {
        return fieldToTag;
    }

    public boolean isScoring() {
        return isScoring;
    }

    public Translation2d getTagToCenterAlign() {
        return tagToCenterAlign;
    }

    public Translation2d getTagToLeftAlign() {
        return tagToLeftAlign;
    }

    public Translation2d getTagToRightAlign() {
        return tagToRightAlign;
    }
}
