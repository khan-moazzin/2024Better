package com.team5817.frc2025.subsystems.Drive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import com.team254.lib.geometry.Pose2d;
import com.team5817.frc2025.field.AlignmentPoint;
import com.team5817.frc2025.field.AprilTag;
import com.team5817.frc2025.field.FieldLayout;
import com.team5817.frc2025.field.AlignmentPoint.AimingRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoAlignPointSelector {
    public static AlignmentPoint a;

 private static Map<Integer, AprilTag> getTagSet() {
        if (DriverStation.getAlliance().isEmpty()) {
            return new HashMap<Integer, AprilTag>();
        }

        Alliance alliance = DriverStation.getAlliance().get();
        if (alliance == Alliance.Red) {
            return FieldLayout.kAprilTagMapRed; // Assuming FieldLayout.Red.kAprilTagMap is defined properly
        } else {
            return FieldLayout.kAprilTagMapBlue; // Assuming FieldLayout.Blue.kAprilTagMap is defined properly
        }
    }


    private static Optional<AprilTag> getNearestTag(Map<Integer, AprilTag> tagMap, Pose2d point,
            AimingRequest desiredAlignment) {
        double closestDistance = Double.MAX_VALUE;
        Optional<AprilTag> closestTag = Optional.empty();

        for (AprilTag tag : tagMap.values()) {
            for (AlignmentPoint alignPoint : tag.getAllAlignmentPoints()) {
                if (alignPoint.getAllowedAllignments().contains(desiredAlignment)) {
                    double distance = tag.getFieldToTag().distance(point);
                    if (distance < closestDistance) {
                        closestDistance = distance;
                        closestTag = Optional.of(tag);
                    }
                    break; // Only need one valid alignment point per tag
                }
            }
        }

        return closestTag;
    }

    private static Pose2d minimizeDistance(Pose2d from, List<Pose2d> candidates, List<AlignmentPoint> points) {
        if (candidates.isEmpty()) {
            return null;
        }

        double closestDistance = Double.MAX_VALUE;
        Pose2d closestPose = candidates.get(0);

        for (int i = 0; i < candidates.size(); i++) {
            double distance = from.distance(candidates.get(i));
            if (distance < closestDistance) {
                closestDistance = distance;
                closestPose = candidates.get(i);
                a = points.get(i);
                Logger.recordOutput("Alignment Point", a.getAllowedAllignments().toString());
            }
        }

        return closestPose;
    }

    private static Pose2d getNearestAlignment(AprilTag tag, Pose2d point, AimingRequest desiredAlignment) {
        List<Pose2d> poses = new ArrayList<>();
        List<AlignmentPoint> points = new ArrayList<>();

        for (AlignmentPoint ap : tag.getAllAlignmentPoints()) {
            if (ap.getAllowedAllignments().contains(desiredAlignment)) {
                Pose2d transformed = tag.getFieldToTag().transformBy(Pose2d.fromTranslation(ap.getTranslation()));
                poses.add(transformed);
                points.add(ap);
            }
        }

        return minimizeDistance(point, poses, points);
    }

    public static Pose2d chooseTargetPoint(Pose2d currentPoint, AimingRequest desiredAlignment) {
        Optional<AprilTag> closestTag = getNearestTag(getTagSet(), currentPoint, desiredAlignment);
        return closestTag.map(tag -> getNearestAlignment(tag, currentPoint, desiredAlignment)).orElse(null);
    }
}
