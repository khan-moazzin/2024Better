package com.team5817.frc2025.subsystems.Drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team5817.frc2025.field.AlignmentPoint;
import com.team5817.frc2025.field.AprilTag;
import com.team5817.frc2025.field.FieldLayout;
import com.team5817.frc2025.field.AlignmentPoint.AlignmentType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoAlignPointSelector {



    private static Map<Integer, AprilTag> getTagSet()  {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return FieldLayout.Red.kAprilTagMap;
        } else {
            return FieldLayout.Blue.kAprilTagMap;
        }
    }

    private static Optional<AprilTag> getNearestTag(Map<Integer, AprilTag> tagMap, Pose2d point, AlignmentType desiredAlignment) {
        double closestDistance = Integer.MAX_VALUE;
        Optional<AprilTag> closestTag = Optional.empty();
        for (int i : tagMap.keySet()) {
            for (AlignmentPoint a : tagMap.get(i).getAllAlignmentPoints()) {
                if (a.getAllowedAllignments().contains(desiredAlignment)) {
                    double distance = tagMap.get(i).getFieldToTag().distance(point);
                    if (distance < closestDistance) {
                        closestDistance = distance;
                        closestTag = Optional.of(tagMap.get(i));
                    }
                    break;
                }
            }
            
        }
        return closestTag;
    }

    private static Pose2d minimizeDistance(Pose2d from, List<AlignmentPoint> to) {
        double closestDistance = Integer.MAX_VALUE;
        Pose2d closestPose = Pose2d.fromTranslation(to.get(0).getTranslation());
        for (int i = 0; i < to.size(); i++) {
            double distance = from.distance(Pose2d.fromTranslation(to.get(i).getTranslation()));
            if (distance <  closestDistance) {
                closestDistance = distance;
                closestPose = Pose2d.fromTranslation(to.get(i).getTranslation());
            }
        }
        return closestPose;
    }

    private static Pose2d getNearestAlignment(AprilTag tag, Pose2d point,AlignmentType desiredAlignment) {
        List<AlignmentPoint> points= new ArrayList<>();
        for(AlignmentPoint a: tag.getAllAlignmentPoints()){
            if(a.getAllowedAllignments().contains(desiredAlignment))
                points.add(a);
            }
        return minimizeDistance(point, points);
    }
    @AutoLogOutput(key = "Nearest Alignment")
    public static Pose2d chooseTargetPoint(Pose2d currentPoint, AlignmentType desiredAlignment) {
        Map<Integer, AprilTag> mTagMap = getTagSet();
        Optional<AprilTag> closestTag = getNearestTag(mTagMap, currentPoint,desiredAlignment);
        Pose2d targetPose = getNearestAlignment(closestTag.get(), currentPoint,desiredAlignment);
        return targetPose;
    }  
}
