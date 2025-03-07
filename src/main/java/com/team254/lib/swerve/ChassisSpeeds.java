// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package com.team254.lib.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.geometry.Twist2d;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.struct.StructSerializable;

/**
 * Represents the speed of a robot chassis. Although this struct contains similar members compared
 * to a Twist2d, they do NOT represent the same thing. Whereas a Twist2d represents a change in pose
 * w.r.t to the robot frame of reference, this ChassisSpeeds struct represents a velocity w.r.t to
 * the robot frame of reference.
 *
 * <p>A strictly non-holonomic drivetrain, such as a differential drive, should never have a dy
 * component because it can never move sideways. Holonomic drivetrains such as swerve and mecanum
 * will often have all three components.
 */
@SuppressWarnings("MemberName")
public class ChassisSpeeds implements StructSerializable{
    /** Represents forward velocity w.r.t the robot frame of reference. (Fwd is +) */
    public double vxMetersPerSecond;

    /** Represents sideways velocity w.r.t the robot frame of reference. (Left is +) */
    public double vyMetersPerSecond;

    /** Represents the angular velocity of the robot frame. (CCW is +) */
    public double omegaRadiansPerSecond;

    /** Constructs a ChassisSpeeds with zeros for dx, dy, and theta. */
    public ChassisSpeeds() {}

    /**
     * Constructs a ChassisSpeeds object.
     *
     * @param vxMetersPerSecond Forward velocity.
     * @param vyMetersPerSecond Sideways velocity.
     * @param omegaRadiansPerSecond Angular velocity.
     */
    public ChassisSpeeds(
            double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        this.vxMetersPerSecond = vxMetersPerSecond;
        this.vyMetersPerSecond = vyMetersPerSecond;
        this.omegaRadiansPerSecond = omegaRadiansPerSecond;
    }
    public ChassisSpeeds(edu.wpi.first.math.kinematics.ChassisSpeeds wpi){
        this.vxMetersPerSecond = wpi.vxMetersPerSecond;
        this.vyMetersPerSecond = wpi.vyMetersPerSecond;
        this.omegaRadiansPerSecond = wpi.omegaRadiansPerSecond;
    }
   


    /**
     * Converts a user provided field-relative set of speeds into a robot-relative ChassisSpeeds
     * object.
     *
     * @param vxMetersPerSecond The component of speed in the x direction relative to the field.
     *     Positive x is away from your alliance wall.
     * @param vyMetersPerSecond The component of speed in the y direction relative to the field.
     *     Positive y is to your left when standing behind the alliance wall.
     * @param omegaRadiansPerSecond The angular rate of the robot.
     * @param robotAngle The angle of the robot as measured by a gyroscope. The robot's angle is
     *     considered to be zero when it is facing directly away from your alliance station wall.
     *     Remember that this should be CCW positive.
     * @return ChassisSpeeds object representing the speeds in the robot's frame of reference.
     */
    public static ChassisSpeeds fromFieldRelativeSpeeds(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            Rotation2d robotAngle) {
        return new ChassisSpeeds(
                vxMetersPerSecond * robotAngle.cos() + vyMetersPerSecond * robotAngle.sin(),
                -vxMetersPerSecond * robotAngle.sin() + vyMetersPerSecond * robotAngle.cos(),
                omegaRadiansPerSecond);
    }

    public static ChassisSpeeds fromFieldRelativeSpeeds(ChassisSpeeds mChassisSpeeds, Rotation2d robotAngle) {
        return fromFieldRelativeSpeeds(
            mChassisSpeeds.vxMetersPerSecond,
            mChassisSpeeds.vyMetersPerSecond,
            mChassisSpeeds.omegaRadiansPerSecond,
            robotAngle
        );
    }

    public static ChassisSpeeds fromRobotRelativeSpeeds(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond) {
        return new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }
    public edu.wpi.first.math.kinematics.ChassisSpeeds wpi(){
        return new edu.wpi.first.math.kinematics.ChassisSpeeds(vxMetersPerSecond,vyMetersPerSecond,omegaRadiansPerSecond);
    }
    public Translation2d getTranslation(){
        return new Translation2d(vxMetersPerSecond,vyMetersPerSecond);
    }
    
  /**
   * Discretizes a continuous-time chassis speed.
   *
   * <p>This function converts a continuous-time chassis speed into a discrete-time one such that
   * when the discrete-time chassis speed is applied for one timestep, the robot moves as if the
   * velocity components are independent (i.e., the robot moves v_x * dt along the x-axis, v_y * dt
   * along the y-axis, and omega * dt around the z-axis).
   *
   * <p>This is useful for compensating for translational skew when translating and rotating a
   * swerve drivetrain.
   *
   * @param vxMetersPerSecond Forward velocity.
   * @param vyMetersPerSecond Sideways velocity.
   * @param omegaRadiansPerSecond Angular velocity.
   * @param dtSeconds The duration of the timestep the speeds should be applied for.
   * @return Discretized ChassisSpeeds.
   */
  public static ChassisSpeeds discretize(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      double dtSeconds) {
    // Construct the desired pose after a timestep, relative to the current pose. The desired pose
    // has decoupled translation and rotation.
    var desiredDeltaPose =
        new Pose2d(
            vxMetersPerSecond * dtSeconds,
            vyMetersPerSecond * dtSeconds,
            new Rotation2d(omegaRadiansPerSecond * dtSeconds,true));

    // Find the chassis translation/rotation deltas in the robot frame that move the robot from its
    // current pose to the desired pose
    var twist = Pose2d.log(desiredDeltaPose);

    // Turn the chassis translation/rotation deltas into average velocities
    return new ChassisSpeeds(twist.dx / dtSeconds, twist.dy / dtSeconds, twist.dtheta / dtSeconds);
  }

  /**
   * Discretizes a continuous-time chassis speed.
   *
   * <p>This function converts a continuous-time chassis speed into a discrete-time one such that
   * when the discrete-time chassis speed is applied for one timestep, the robot moves as if the
   * velocity components are independent (i.e., the robot moves v_x * dt along the x-axis, v_y * dt
   * along the y-axis, and omega * dt around the z-axis).
   *
   * <p>This is useful for compensating for translational skew when translating and rotating a
   * swerve drivetrain.
   *
   * @param vx Forward velocity.
   * @param vy Sideways velocity.
   * @param omega Angular velocity.
   * @param dt The duration of the timestep the speeds should be applied for.
   * @return Discretized ChassisSpeeds.
   */
  public static ChassisSpeeds discretize(
      LinearVelocity vx, LinearVelocity vy, AngularVelocity omega, Time dt) {
    return discretize(
        vx.in(MetersPerSecond), vy.in(MetersPerSecond), omega.in(RadiansPerSecond), dt.in(Seconds));
  }

  /**
   * Discretizes a continuous-time chassis speed.
   *
   * <p>This function converts a continuous-time chassis speed into a discrete-time one such that
   * when the discrete-time chassis speed is applied for one timestep, the robot moves as if the
   * velocity components are independent (i.e., the robot moves v_x * dt along the x-axis, v_y * dt
   * along the y-axis, and omega * dt around the z-axis).
   *
   * <p>This is useful for compensating for translational skew when translating and rotating a
   * swerve drivetrain.
   *
   * @param continuousSpeeds The continuous speeds.
   * @param dtSeconds The duration of the timestep the speeds should be applied for.
   * @return Discretized ChassisSpeeds.
   */
  public static ChassisSpeeds discretize(ChassisSpeeds continuousSpeeds, double dtSeconds) {
    return discretize(
        continuousSpeeds.vxMetersPerSecond,
        continuousSpeeds.vyMetersPerSecond,
        continuousSpeeds.omegaRadiansPerSecond,
        dtSeconds);
  }

    public Twist2d toTwist2d() {
        return new Twist2d(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }

    @Override
    public String toString() {
        return String.format(
                "ChassisSpeeds(Vx: %.2f m/s, Vy: %.2f m/s, Omega: %.2f rad/s)",
                vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    }
    public static final ChassisSpeedsStruct struct = new ChassisSpeedsStruct();
}

