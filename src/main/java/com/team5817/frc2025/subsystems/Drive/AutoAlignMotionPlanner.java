package com.team5817.frc2025.subsystems.Drive;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.motion.IMotionProfileGoal;
import com.team254.lib.motion.MotionProfileGoal;
import com.team254.lib.motion.MotionState;
import com.team254.lib.motion.ProfileFollower;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team254.lib.util.Units;
import com.team5817.frc2025.Constants;
import com.team5817.frc2025.RobotState;
import com.team5817.frc2025.Constants.SwerveConstants;
import com.team5817.lib.swerve.SwerveHeadingController;

import edu.wpi.first.wpilibj.Timer;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

/**
 * Class responsible for planning the motion for auto-alignment.
 */
public class AutoAlignMotionPlanner {

    private ProfileFollower mYController = new ProfileFollower(4, 10, 0.0, 0.0, 0.0, 0.0);
    private ProfileFollower mXController = new ProfileFollower(4, 10, 0.0, 0.0, 0.0, 0.0);
    private SwerveHeadingController mThetaController;

    boolean mAutoAlignComplete = false;

    private Pose2d mFieldToTargetPoint;
    private Pose2d poseDeadband;
    private OptionalDouble mStartTime;
    private Translation2d error = new Translation2d();

    /**
     * Constructor for AutoAlignMotionPlanner.
     */
    public AutoAlignMotionPlanner() {
        mThetaController = new SwerveHeadingController();
        
    }

    /**
     * Resets the motion planner to its initial state.
     */
    public void reset() {
        mStartTime = OptionalDouble.of(Timer.getFPGATimestamp());
        mXController.resetProfile();
        mXController.resetIntegral();
        mXController.resetSetpoint();
        mYController.resetProfile();
        mYController.resetSetpoint();
        mYController.resetIntegral();
        mAutoAlignComplete = false;
    }

    /**
     * Sets the target point for auto-alignment.
     * 
     * @param targetPoint The target point to align to.
     */
    public void setTargetPoint(Pose2d targetPoint, Pose2d poseDeadband) {
        mFieldToTargetPoint = targetPoint;
        mXController.resetIntegral();
        mYController.resetIntegral();
        this.poseDeadband = Pose2d.fromTranslation(poseDeadband.getTranslation().rotateBy(targetPoint.getRotation())).withRotation(poseDeadband.getRotation());
        Logger.recordOutput("Align Point",new edu.wpi.first.math.geometry.Pose2d(mFieldToTargetPoint.getTranslation().wpi(),mFieldToTargetPoint.getRotation().wpi()));
    }

    /**
     * Updates the motion planner with the current timestamp, pose, and velocity.
     * 
     * @param timestamp    The current timestamp.
     * @param current_pose The current pose of the robot.
     * @param current_vel  The current velocity of the robot.
     * @return The updated chassis speeds.
     */
    public ChassisSpeeds updateAutoAlign(double timestamp, Pose2d current_pose, Twist2d current_vel) {
        mXController.setGoalAndConstraints(
                new MotionProfileGoal(mFieldToTargetPoint.getTranslation().x(), 2,
                        IMotionProfileGoal.CompletionBehavior.VIOLATE_MAX_ACCEL, Math.abs(poseDeadband.getTranslation().x()), 0.1),
                SwerveConstants.kPositionMotionProfileConstraints);
        mYController.setGoalAndConstraints(
                new MotionProfileGoal(mFieldToTargetPoint.getTranslation().y(), 2,
                        IMotionProfileGoal.CompletionBehavior.VIOLATE_MAX_ACCEL, Math.abs(poseDeadband.getTranslation().y()), 0.1),
                SwerveConstants.kPositionMotionProfileConstraints);
        mThetaController.setSnapTarget(mFieldToTargetPoint.getRotation());
        double currentRotation = current_pose.getRotation().getRadians();

        if (mFieldToTargetPoint.getRotation().getRadians() - currentRotation > Math.PI) {
            currentRotation += 2 * Math.PI;
        } else if (mFieldToTargetPoint.getRotation().getRadians() - currentRotation < -Math.PI) {
            currentRotation -= 2 * Math.PI;
        }
        Translation2d vel = new Translation2d(current_vel.dx,current_vel.dy);
        double xOutput = mXController.update(
                new MotionState(timestamp, current_pose.getTranslation().x(), vel.x(), 0.0),
                timestamp + Constants.kLooperDt);
        double yOutput = mYController.update(
                new MotionState(timestamp, current_pose.getTranslation().y(),vel.y() , 0.0),
                timestamp + Constants.kLooperDt);
        double thetaOutput = mThetaController.update(current_pose.getRotation(), timestamp);
        ChassisSpeeds setpoint = new ChassisSpeeds();

        this.error = current_pose.minus(mFieldToTargetPoint).getTranslation();
        boolean thetaWithinDeadband =  current_pose.getRotation().distance(mFieldToTargetPoint.getRotation()) < poseDeadband.getRotation().getRadians() && Math.abs(thetaOutput) < 0.02;
        boolean xWithinDeadband = mXController.onTarget();
        boolean yWithinDeadband = mYController.onTarget();
        if(mAutoAlignComplete)
        setpoint = ChassisSpeeds.fromFieldRelativeSpeeds(
             0.0,
             0.0,
             0.0,
            current_pose.getRotation());
        setpoint = ChassisSpeeds.fromFieldRelativeSpeeds(
                xWithinDeadband ? 0.0 : xOutput,
                yWithinDeadband ? 0.0 : yOutput,
                thetaWithinDeadband ? 0.0 : thetaOutput,
                current_pose.getRotation());
        mAutoAlignComplete = thetaWithinDeadband && xWithinDeadband && yWithinDeadband;
        Logger.recordOutput("AutoAlign/xDone", xWithinDeadband);
        Logger.recordOutput("AutoAlign/yDone", yWithinDeadband);
        Logger.recordOutput("AutoAlign/tDone", thetaWithinDeadband);


        if (mStartTime.isPresent() && mAutoAlignComplete) {
            System.out.println("Auto align took: " + (Timer.getFPGATimestamp() - mStartTime.getAsDouble()));
            mStartTime = OptionalDouble.empty();
        }
        return setpoint;
    }

    /**
     * Checks if the auto-alignment is complete.
     * 
     * @return True if auto-alignment is complete, false otherwise.
     */
    public boolean getAutoAlignComplete() {
        return mAutoAlignComplete;
    }
    public Translation2d getAutoAlignError(){
        return this.error;
    }
}
