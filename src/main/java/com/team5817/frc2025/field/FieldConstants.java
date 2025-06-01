package com.team5817.frc2025.field;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team5817.lib.util.Doubles;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.util.Units;

import java.util.List;

public final class FieldConstants {

    public static final double kFieldLength = Units.inchesToMeters(651.223);
    public static final double kFieldWidth = Units.inchesToMeters(323.277);

    public static final double kSpeakerHeight = 2.07;
    public static final double kAllianceWallX = 0.0;
    public static final double kFarWallX = kFieldLength;

    /** Obstacles on field for pathfinding or collision avoidance */
    
    
    
    public static Pose2d getSpeakerAimingPose() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return alliance == Alliance.Red
            ? new Pose2d(16.7, 5.56, new Rotation2d())
            : new Pose2d(0.42, 5.56, new Rotation2d()); // 16.7 - 16.28
    }

    public static Pose2d getSpeakerPivotPose() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return alliance == Alliance.Red
            ? new Pose2d(16.5, 5.56, new Rotation2d())
            : new Pose2d(0.0, 5.56, new Rotation2d());
    }

    public static Pose2d getLobPose() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        return alliance == Alliance.Red
            ? new Pose2d(15.77, 7.43, new Rotation2d())
            : new Pose2d(0.73, 7.43, new Rotation2d()); // 16.5 - 15.77
    }
    
    
    
    
    
    public static final class Obstacles {
        private static final double[] boxX = Doubles.of(2.88, 5.83, 5.83);
        private static final double[] boxY = Doubles.of(4.1, 5.73, 2.47);

        private static double[] reflect(double[] xs) {
            double[] reflected = new double[xs.length];
            for (int i = 0; i < xs.length; i++) {
                reflected[i] = 16.5 - xs[i]; // Reflect across centerline
            }
            return reflected;
        }

        public static final List<List<Translation2d>> kObstacles = List.of(
            List.of(
                new Translation2d(boxX[0], boxY[0]),
                new Translation2d(boxX[1], boxY[1]),
                new Translation2d(boxX[2], boxY[2])
            ),
            List.of(
                new Translation2d(reflect(boxX)[0], boxY[0]),
                new Translation2d(reflect(boxX)[1], boxY[1]),
                new Translation2d(reflect(boxX)[2], boxY[2])
            )
        );

    }

    /** Static field element poses (like speaker & amp) */
    public static final class FieldElements {

        private static final double kAmpY = 8.19;
        private static final double kBlueAmpX = 1.79;
        private static final double kRedAmpX = 16.5;

        public static Pose2d getAmpPose() {
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            return alliance == Alliance.Red
                ? new Pose2d(kRedAmpX, kAmpY, new Rotation2d())
                : new Pose2d(kBlueAmpX, kAmpY, new Rotation2d());
        }

        public static final Pose2d kSpeakerBlue = new Pose2d(
            0.2, 
            kFieldWidth - Units.inchesToMeters(104.0), 
            new Rotation2d()
        );
    }

    private FieldConstants() {} // Prevent instantiation
}
