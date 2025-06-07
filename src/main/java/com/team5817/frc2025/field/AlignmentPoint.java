package com.team5817.frc2025.field;

import java.util.List;
import java.util.Optional;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;
import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;
import com.team5817.frc2025.RobotState;
import com.team5817.frc2025.RobotState.VisionUpdate;
import com.team5817.frc2025.subsystems.Shooter.ShooterConstants;
import com.team5817.lib.swerve.SwerveHeadingController;

/**
 * Represents an alignment point on the field with a specific pose and allowed alignments.
 */
public class AlignmentPoint {

    private List<AimingRequest> mAllowedAllignments;
    private Translation2d mPose;

    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShotTimeMap = ShooterConstants.SHOT_TRAVEL_TIME_TREE_MAP;


    /**
     * Constructs an AlignmentPoint with the specified pose and allowed alignments.
     *
     * @param pose the position of the alignment point
     * @param allowed_allignments the types of alignments allowed at this point
     */
    public AlignmentPoint (Translation2d pose, AimingRequest... allowed_allignments){
        this.mPose = pose;
        this.mAllowedAllignments = List.of(allowed_allignments);
    }

    /**
     * Enum representing the types of alignments that can be allowed at an alignment point.
     */
    public enum AimingRequest{
        SPEAKER,
        LOB;
      
        public Pose2d tolerance;
        private AimingRequest(Pose2d tolerance){
            this.tolerance = tolerance;
        }
        private AimingRequest(){
            this.tolerance = new Pose2d(.04, 0.04, 3);
        }
    }

    /**
     * Gets the translation (position) of the alignment point.
     *
     * @return the translation of the alignment point
     */
    public Translation2d getTranslation(){
        return mPose;
    }

    /**
     * Gets the list of allowed alignments at the alignment point.
     *
     * @return the list of allowed alignments
     */
    public List<AimingRequest> getAllowedAllignments(){
        return mAllowedAllignments;
    }
    
    private Pose2d mFieldToSpeaker;
    private AimingRequest mAimingRequest;
    private boolean isAimed = false;

    public Pose2d updateAiming(double timeStamp, Pose2d currentOdomToRobot, Pose2d visionPoseComponent,
            AimingRequest request, Optional<VisionUpdate> visionUpdate, SwerveHeadingController headingController,
            Twist2d currentVelocity) {
        Pose2d targetPose = new Pose2d();
        mAimingRequest = request;
        switch (mAimingRequest) {
            case SPEAKER:
                mFieldToSpeaker = FieldConstants.getSpeakerAimingPose();
                break;
            case LOB:
                mFieldToSpeaker = FieldConstants.getLobPose();
                break;
        }
        double estimatedTimeFrame = 0;
        Pose2d odomToTargetPoint = visionPoseComponent.inverse().transformBy(mFieldToSpeaker);
        double travelDistance = odomToTargetPoint.transformBy(currentOdomToRobot).getTranslation().norm();
        estimatedTimeFrame = mShotTimeMap.getInterpolated(new InterpolatingDouble(travelDistance)).value;
        Pose2d poseAtTimeFrame = RobotState.getInstance().getPredictedPoseFromOdometry(estimatedTimeFrame);

        Pose2d futureOdomToTargetPoint = poseAtTimeFrame.inverse().transformBy(odomToTargetPoint).inverse();
        Rotation2d targetRotation = futureOdomToTargetPoint.getTranslation().getAngle().inverse();
        targetPose = new Pose2d(futureOdomToTargetPoint.getTranslation(), targetRotation);
        headingController.setTargetHeading(targetPose.getRotation().inverse());
        double rotationOutput = headingController.updateRotationCorrection(currentOdomToRobot.getRotation().inverse(),
                timeStamp);
        isAimed = headingController.atTarget();
        targetPose = new Pose2d(
                targetPose.getTranslation(),
                Rotation2d.fromDegrees(rotationOutput * 1.75));

        return targetPose;
    }

    public boolean isAimed() {
        return isAimed;
    }


}
