package com.team5817.frc2024.field;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import java.io.IOException;
import java.util.HashMap;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in
 * Meters</b> <br>
 * <br>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldLayout {
	public static double kFieldLength = Units.inchesToMeters(651.223); //TODO
	public static double kFieldWidth = Units.inchesToMeters(323.277); //TODO

	public static final double kApriltagWidth = Units.inchesToMeters(6.50); //TODO
	public static final AprilTagFieldLayout kTagMap;

	static {
		try {
			kTagMap = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile); //TODO
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}

    public static class Red {
        public static final HashMap<Integer, AprilTag> kAprilTagMap = new HashMap<>();
        // From field manual: 46.355 cm
        // From CAD: 46.2534 cm
        // From step layout marking: 46.2788 cm
        // Old value: 46.272  cm
        private static final Translation2d kTag1ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag1ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag1ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag2ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag2ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag2ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag3ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag3ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag3ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag5ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag5ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag5ToLeftAlign = new Translation2d(0.77, 0.55);
        public static final AprilTag kAprilTag1 = new AprilTag(
                1,
                0.462534,
                new Pose2d(1.027, 6.94659, Rotation2d.fromDegrees(0)),
                true,
                kTag1ToCenterAlign,
                kTag1ToLeftAlign,
                kTag1ToRightAlign
        );
        public static final AprilTag kAprilTag2 = new AprilTag(
                2,
                0.462534,
                new Pose2d(1.027, 5.27019, Rotation2d.fromDegrees(0)),
                true,
                kTag2ToCenterAlign,
                kTag2ToLeftAlign,
                kTag2ToRightAlign);
        public static final AprilTag kAprilTag3 = new AprilTag(
                3,
                0.462534,
                new Pose2d(1.027, 3.59379, Rotation2d.fromDegrees(0)),
                true,
                kTag3ToCenterAlign,
                kTag3ToLeftAlign,
                kTag3ToRightAlign);
        public static final AprilTag kAprilTag5 = new AprilTag(
                5,
                0.695452,
                new Pose2d(16.17832, 1.26839, Rotation2d.fromDegrees(0)),
                false,
                kTag5ToCenterAlign,
                kTag5ToLeftAlign,
                kTag5ToRightAlign);


        static {
            kAprilTagMap.put(1, kAprilTag1);
            kAprilTagMap.put(2, kAprilTag2);
            kAprilTagMap.put(3, kAprilTag3);
            kAprilTagMap.put(5, kAprilTag5);
        }


    }

    public static class Blue {
        public static final HashMap<Integer, AprilTag> kAprilTagMap = new HashMap<>();
        // From field manual: 46.355 cm
        // From CAD: 46.2534 cm
        // From step layout marking: 46.2788 cm
        // Old value: 46.272  cm
        private static final Translation2d kTag8ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag8ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag8ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag7ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag7ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag7ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag6ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag6ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag6ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag4ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag4ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag4ToLeftAlign = new Translation2d(0.77, 0.55);
        public static final AprilTag kAprilTag8 = new AprilTag(
                8,
                0.462534,
                new Pose2d(1.027, 1.07341, Rotation2d.fromDegrees(0)),
                true,
                kTag8ToCenterAlign,
                kTag8ToLeftAlign,
                kTag8ToRightAlign
        );
        public static final AprilTag kAprilTag7 = new AprilTag(
                7,
                0.462534,
                new Pose2d(1.027, 2.74981, Rotation2d.fromDegrees(0)),
                true,
                kTag7ToCenterAlign,
                kTag7ToLeftAlign,
                kTag7ToRightAlign);
        public static final AprilTag kAprilTag6 = new AprilTag(
                6,
                0.462534,
                new Pose2d(1.027, 4.4221, Rotation2d.fromDegrees(0)),
                true,
                kTag6ToCenterAlign,
                kTag6ToLeftAlign,
                kTag6ToRightAlign);
        public static final AprilTag kAprilTag4 = new AprilTag(
                4,
                0.695452,
                new Pose2d(16.17832, 6.75161, Rotation2d.fromDegrees(0)),
                false,
                kTag4ToCenterAlign,
                kTag4ToLeftAlign,
                kTag4ToRightAlign);


        static {
            kAprilTagMap.put(4, kAprilTag4);
            kAprilTagMap.put(6, kAprilTag6);
            kAprilTagMap.put(7, kAprilTag7);
            kAprilTagMap.put(8, kAprilTag8);
        }
    }
	
	public static Pose2d handleAllianceFlip(Pose2d blue_pose, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_pose = blue_pose.mirrorAboutX(kFieldLength / 2.0);
		}
		return blue_pose;
	}

	public static Translation2d handleAllianceFlip(Translation2d blue_translation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_translation = blue_translation.mirrorAboutX(kFieldLength / 2.0);
		}
		return blue_translation;
	}

	public static Rotation2d handleAllianceFlip(Rotation2d blue_rotation, boolean is_red_alliance) {
		if (is_red_alliance) {
			blue_rotation = blue_rotation.mirrorAboutX();
		}
		return blue_rotation;
	}

	public static double distanceFromAllianceWall(double x_coordinate, boolean is_red_alliance) {
		if (is_red_alliance) {
			return kFieldLength - x_coordinate;
		}
		return x_coordinate;
	}
}
