package com.team5817.frc2025.subsystems.Drive;

import java.util.Map;
import java.util.Optional;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team5817.frc2025.field.AprilTag;
import com.team5817.frc2025.field.FieldLayout;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoAlignPointSelector {


    public enum RequestedAlignment {
        CENTER, // force center (will do nothing if feeder)
        RIGHT, // force right
        LEFT, // force left
        AUTO_CORAL, // go to whatever position is closest that can score cubes mid or high
        AUTO_ALGAE, // go to whatever position is closest that can score cones mid or high

        AUTO, // go to whatever position is closest
    }

    private static Map<Integer, AprilTag> getTagSet()  {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return FieldLayout.Red.kAprilTagMap;
        } else {
            return FieldLayout.Blue.kAprilTagMap;
        }
    }

    private static Optional<AprilTag> getNearestTag(Map<Integer, AprilTag> tagMap, Pose2d point) {
        double closestDistance = Integer.MAX_VALUE;
        Optional<AprilTag> closestTag = Optional.empty();
        for (int i : tagMap.keySet()) {
            double distance = tagMap.get(i).getFieldToTag().distance(point);
            if (distance <  closestDistance) {
                closestDistance = distance;
                closestTag = Optional.of(tagMap.get(i));
            }
        }
        return closestTag;
    }

    private static Optional<Pose2d> minimizeDistance(Pose2d from, Pose2d[] to) {
        if (to.length == 0) {
            return Optional.empty();
        }
        double closestDistance = Integer.MAX_VALUE;
        Pose2d closestPose = to[0];
        for (int i = 0; i < to.length; i++) {
            double distance = from.distance(to[i]);
            if (distance <  closestDistance) {
                closestDistance = distance;
                closestPose = to[i];
            }
        }
        return Optional.of(closestPose);
    }

    private static Optional<Pose2d> getNearestAlignment(AprilTag tag, Pose2d point, boolean coral, boolean algae) {
        if (tag.isScoring()) {
            Pose2d center = tag.getFieldToTag().transformBy(Pose2d.fromTranslation(tag.getTagToCenterAlign()));
            Pose2d left = tag.getFieldToTag().transformBy(Pose2d.fromTranslation(tag.getTagToLeftAlign()));
            Pose2d right = tag.getFieldToTag().transformBy(Pose2d.fromTranslation(tag.getTagToRightAlign()));
            if (coral) {
                return minimizeDistance(point, new Pose2d[]{left,right});
            } else if (algae) {
                return Optional.of(center);
            } else {
                return minimizeDistance(point, new Pose2d[]{left,center,right});
            }
        } else {
            Pose2d left = tag.getFieldToTag().transformBy(Pose2d.fromTranslation(tag.getTagToLeftAlign()));
            Pose2d right = tag.getFieldToTag().transformBy(Pose2d.fromTranslation(tag.getTagToRightAlign()));
            return minimizeDistance(point, new Pose2d[]{left,right});
        }
    }

    public static Optional<Pose2d> chooseTargetPoint(Pose2d currentPoint, RequestedAlignment alignment) {
        Map<Integer, AprilTag> mTagMap = getTagSet();
        Optional<AprilTag> closestTag = getNearestTag(mTagMap, currentPoint);
        if (closestTag.isEmpty()) {
            return Optional.empty();
        }
        Optional<Pose2d> targetPose = Optional.empty();
        if (alignment == RequestedAlignment.AUTO || alignment == RequestedAlignment.AUTO_CORAL || alignment == RequestedAlignment.AUTO_ALGAE) {
            targetPose = getNearestAlignment(closestTag.get(), currentPoint,
                    alignment == RequestedAlignment.AUTO || alignment == RequestedAlignment.AUTO_CORAL,
                    alignment == RequestedAlignment.AUTO || alignment == RequestedAlignment.AUTO_ALGAE);
        } else {
            if (closestTag.get().isScoring()) {
                switch (alignment) {
                    case CENTER:
                        targetPose = Optional.of(closestTag.get().getFieldToTag().transformBy(Pose2d.fromTranslation(closestTag.get().getTagToCenterAlign())));
                        break;
                    case LEFT:
                        targetPose = Optional.of(closestTag.get().getFieldToTag().transformBy(Pose2d.fromTranslation(closestTag.get().getTagToLeftAlign())));
                        break;
                    case RIGHT:
                        targetPose = Optional.of(closestTag.get().getFieldToTag().transformBy(Pose2d.fromTranslation(closestTag.get().getTagToRightAlign())));
                        break;
                }
                targetPose = Optional.of(new Pose2d(targetPose.get().getTranslation(), Rotation2d.fromDegrees(180)));
            } else {
                switch (alignment) {
                    case LEFT:
                        targetPose = Optional.of(closestTag.get().getFieldToTag().transformBy(Pose2d.fromTranslation(closestTag.get().getTagToLeftAlign())));
                        break;
                    case RIGHT:
                        targetPose = Optional.of(closestTag.get().getFieldToTag().transformBy(Pose2d.fromTranslation(closestTag.get().getTagToRightAlign())));
                        break;
                    default:
                        // center align in feeder is useless
                        return Optional.empty();
                }
                targetPose = Optional.of(new Pose2d(targetPose.get().getTranslation(), Rotation2d.fromDegrees(0)));
            }
        }
        return targetPose;
    }  
}
