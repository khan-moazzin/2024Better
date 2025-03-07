package com.team5817.frc2025.field;

import java.util.ArrayList;
import java.util.List;

import com.team254.lib.geometry.Pose2d;
import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;

/**
 * Represents an AprilTag on the field.
 */
public class AprilTag {

    private int id;
    private Pose2d fieldToTag;
    private boolean isScoring;

    private AlignmentPoint tagToLeftAlign;
    private AlignmentPoint tagToRightAlign;
    private AlignmentPoint tagToCenterAlign;
    private List<AlignmentType> allTypes = new ArrayList<>();
    private List<AlignmentPoint> allAlignmentPoints;

    /**
     * Constructs an AprilTag with the specified parameters.
     *
     * @param id the ID of the AprilTag
     * @param isScoring whether the AprilTag is for scoring
     * @param tagToCenterAlign the center alignment point
     * @param tagToLeftAlign the left alignment point
     * @param tagToRightAlign the right alignment point
     */
    public AprilTag(int id, boolean isScoring, AlignmentPoint tagToCenterAlign, AlignmentPoint tagToLeftAlign, AlignmentPoint tagToRightAlign) {
        this.id = id;
        this.isScoring = isScoring;
        this.tagToCenterAlign = tagToCenterAlign;
        this.tagToLeftAlign = tagToLeftAlign;
        this.tagToRightAlign = tagToRightAlign;
        this.fieldToTag = new Pose2d(FieldLayout.kTagMap.getTagPose(id).get().toPose2d());
        this.allAlignmentPoints = List.of(tagToCenterAlign, tagToLeftAlign, tagToRightAlign);

        if (isScoring) {
            this.fieldToTag = fieldToTag.withRotation(fieldToTag.getRotation().flip());
        }
        for (AlignmentPoint alignments : allAlignmentPoints) {
            for (AlignmentType tagTypes : alignments.getAllowedAllignments()) {
                if (!allTypes.contains(tagTypes)) {
                    allTypes.add(tagTypes);
                }
            }
        }
    }

    /**
     * Gets the ID of the AprilTag.
     *
     * @return the ID of the AprilTag
     */
    public int getId() {
        return id;
    }

    /**
     * Gets the pose of the AprilTag relative to the field.
     *
     * @return the pose of the AprilTag relative to the field
     */
    public Pose2d getFieldToTag() {
        return fieldToTag;
    }

    /**
     * Checks if the AprilTag is for scoring.
     *
     * @return true if the AprilTag is for scoring, false otherwise
     */
    public boolean isScoring() {
        return isScoring;
    }

    /**
     * Gets all allowable alignment types for the AprilTag.
     *
     * @return a list of allowable alignment types
     */
    public List<AlignmentType> getAllAllowableAllignments() {
        return allTypes;
    }

    /**
     * Gets all alignment points for the AprilTag.
     *
     * @return a list of alignment points
     */
    public List<AlignmentPoint> getAllAlignmentPoints() {
        return allAlignmentPoints;
    }

    /**
     * Gets the center alignment point for the AprilTag.
     *
     * @return the center alignment point
     */
    public AlignmentPoint getTagToCenterAlign() {
        return tagToCenterAlign;
    }

    /**
     * Gets the left alignment point for the AprilTag.
     *
     * @return the left alignment point
     */
    public AlignmentPoint getTagToLeftAlign() {
        return tagToLeftAlign;
    }

    /**
     * Gets the right alignment point for the AprilTag.
     *
     * @return the right alignment point
     */
    public AlignmentPoint getTagToRightAlign() {
        return tagToRightAlign;
    }
}
