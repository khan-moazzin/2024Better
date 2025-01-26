package com.team5817.frc2025.field;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;

public class AprilTag {

    private int id;
    private Pose2d fieldToTag;
    private boolean isScoring;

    private AlignmentPoint tagToLeftAlign;

    private AlignmentPoint tagToRightAlign;

    private AlignmentPoint tagToCenterAlign;
    private List<AlignmentType> allTypes = new ArrayList<>();
    private List<AlignmentPoint> allAlignmentPoints;

    public AprilTag(int id, boolean isScoring, AlignmentPoint tagToCenterAlign, AlignmentPoint tagToLeftAlign, AlignmentPoint tagToRightAlign) {
        this.id = id;
        this.isScoring = isScoring;
        this.tagToCenterAlign = tagToCenterAlign;
        this.tagToLeftAlign = tagToLeftAlign;
        this.tagToRightAlign = tagToRightAlign;
        this.fieldToTag = new Pose2d(FieldLayout.kTagMap.getTagPose(id).get().toPose2d());
        this.allAlignmentPoints = List.of(tagToCenterAlign,tagToLeftAlign,tagToRightAlign);

        if(isScoring){
            this.fieldToTag = fieldToTag.withRotation(fieldToTag.getRotation().flip());
        }
        for (AlignmentPoint alingments : allAlignmentPoints) {
            for (AlignmentType tagTypes : alingments.getAllowedAllignments()) {
                if(!allTypes.contains(tagTypes)){
                    allTypes.add(tagTypes);
                }
            }
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

    public List<AlignmentType> getAllAllowableAllignments(){
        return allTypes;
    }

    public List<AlignmentPoint> getAllAlignmentPoints(){
        return allAlignmentPoints;
    }

    public AlignmentPoint getTagToCenterAlign() {
        return tagToCenterAlign;
    }

    public AlignmentPoint getTagToLeftAlign() {
        return tagToLeftAlign;
    }

    public AlignmentPoint getTagToRightAlign() {
        return tagToRightAlign;
    }
}
