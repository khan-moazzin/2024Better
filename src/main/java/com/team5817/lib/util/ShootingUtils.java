package com.team5817.lib.util;

import org.littletonrobotics.junction.Logger;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team254.lib.util.InterpolatingDouble;
import com.team5817.frc2025.RobotState;
import com.team5817.frc2025.subsystems.Pivot.PivotConstants;
import com.team5817.frc2025.subsystems.Shooter.ShooterConstants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class ShootingUtils {
    

    public enum NoteState {
        NEW,
        MEDIUM,
        OLD
    }

    public static class ShootingParameters{
        public double effectiveDistance;
        public double compensatedDistance;
        public double spin;

        public double uncompensatedDesiredPivotAngle;
        public double compensatedDesiredPivotAngle;

        public double uncompensatedDesiredShooterSpeed;
        public double compensatedDesiredShooterSpeed;



        public ShootingParameters(
           double effectiveDistance,
           double compensatedDistance,
           double compensatedDesiredPivotAngle,
           double uncompensatedDesiredPivotAngle,

           double compensatedDesiredShooterSpeed,
           double uncompensatedDesiredShooterSpeed,
           double spin

        ){
         this.spin = spin;
         this.effectiveDistance = effectiveDistance;
         this.compensatedDistance = compensatedDistance;
         this.compensatedDesiredPivotAngle = compensatedDesiredPivotAngle;
         this.uncompensatedDesiredPivotAngle = uncompensatedDesiredPivotAngle;
         this.compensatedDesiredShooterSpeed = compensatedDesiredShooterSpeed;
         this.uncompensatedDesiredShooterSpeed = uncompensatedDesiredShooterSpeed;


        }
    }


    public static ShootingParameters getShootingParameters(
        double pivotOffset,
        Pose2d currentPose, 
        Pose2d targetPose,
        double kShotTime, 
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotAngleTreeMap,
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> velocityTreeMap,
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shotTimeTreeMap,
        Twist2d currentVelocity,
        boolean manual){
        Pose2d robotToTarget = Pose2d.fromTranslation(targetPose.getTranslation().translateBy(currentPose.getTranslation().inverse()));
        double effectiveDistance = robotToTarget.getTranslation().norm();
        double lookahead_time = shotTimeTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        Pose2d poseAtTimeFrame = RobotState.getInstance().getPredictedPose(lookahead_time*1.3);
        Translation2d futureOdomToTargetPoint = poseAtTimeFrame.getTranslation().translateBy(targetPose.getTranslation().inverse());

        double compensatedDistance = futureOdomToTargetPoint.norm();

        
        double compensatedPivotAngle;
;
        if(manual){
            compensatedPivotAngle = 21+pivotOffset;
        }
        else{
            compensatedPivotAngle = pivotAngleTreeMap.getInterpolated(new InterpolatingDouble(compensatedDistance)).value+pivotOffset;
        }
       Logger.recordOutput("compensatedDistance", compensatedDistance);
        double desiredPivotAngle = pivotAngleTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        double uncompensatedDesiredShooterSpeed = velocityTreeMap.getInterpolated(new InterpolatingDouble(effectiveDistance)).value;
        double compensatedSpin = ShooterConstants.SPIN_TREE_MAP.getInterpolated(new InterpolatingDouble(compensatedDistance)).value;
        double compensatedDesiredShooterSpeed = velocityTreeMap.getInterpolated(new InterpolatingDouble(compensatedDistance)).value;

        if(compensatedPivotAngle>36){
            compensatedPivotAngle = 36;
        }
        else if(compensatedPivotAngle<1){
            compensatedPivotAngle = 1;
        }
        return new ShootingParameters(
            effectiveDistance, 
            compensatedDistance, 
            compensatedPivotAngle,
            desiredPivotAngle,
            compensatedDesiredShooterSpeed, 
            uncompensatedDesiredShooterSpeed,
            compensatedSpin

            );
    }



    public static InterpolatingDoubleTreeMap getPivotMap(boolean lob){

        if
        (lob)
            return PivotConstants.LobAngleMap;
        return PivotConstants.SpeakerAngleMap;
    }
 public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getVelocityMap(boolean lob){
        if(lob)
            return ShooterConstants.LOB_VELOCITY_TREE_MAP;
        return ShooterConstants.SPEAKER_VELOCITY_TREE_MAP;
    }
}
