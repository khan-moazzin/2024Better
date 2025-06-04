package com.team5817.lib.util;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team5817.frc2025.subsystems.Shooter.ShooterConstants;
import com.team5817.lib.util.ShootingUtils.ShootingParameters;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;



public class ShootingParametersBuilder {
    private double pivotOffset = 0.0;
    private Pose2d currentPose;
    private Pose2d targetPose;
    private double shotTime = ShooterConstants.kShotTime;
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> pivotMap;
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> velocityMap;
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> shotTimeMap = ShooterConstants.SHOT_TRAVEL_TIME_TREE_MAP;
    private Twist2d currentVelocity;
    private boolean manual = false;

    public ShootingParametersBuilder setPivotOffset(double offset) {
        this.pivotOffset = offset;
        return this;
    }

    public ShootingParametersBuilder setCurrentPose(Pose2d currentPose) {
        this.currentPose = currentPose;
        return this;
    }

    public ShootingParametersBuilder setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
        return this;
    }

    public ShootingParametersBuilder setPivotMap(InterpolatingDoubleTreeMap map) {
        this.pivotMap = convert254MapToInterpolatingTreeMap(map);
        return this;
    }

    public ShootingParametersBuilder setVelocityMap(InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> map) {
        this.velocityMap = map;
        return this;
    }

    public ShootingParametersBuilder setShotTimeMap(InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> map) {
        this.shotTimeMap = map;
        return this;
    }

    public ShootingParametersBuilder setCurrentVelocity(Twist2d velocity) {
        this.currentVelocity = velocity;
        return this;
    }

    public ShootingParametersBuilder setManual(boolean manual) {
        this.manual = manual;
        return this;
    }

    
    
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> convert254MapToInterpolatingTreeMap(InterpolatingDoubleTreeMap map) {
        InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> newMap = new InterpolatingTreeMap<>();
        for (double x = 0.0; x <= 100.0; x += 0.1) { // Adjust range as needed
            if (map.get(x) != null) {
                newMap.put(new InterpolatingDouble(x), new InterpolatingDouble(map.get(x)));
            }
        }
        return newMap;
    }
    
    
    
    
    
    public ShootingParameters build() {
        return ShootingUtils.getShootingParameters(
            pivotOffset,
            currentPose,
            targetPose,
            shotTime,
            pivotMap,
            velocityMap,
            shotTimeMap,
            currentVelocity,
            manual
        );
    }
}
