package com.team5817.frc2025;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class RobotVisualizer {
    public static Pose3d[] mechanismPoses = new Pose3d[6];
    static {
    for (int i = 0; i < 6; i++) {
      mechanismPoses[i] = new Pose3d();
    }}

  public static void outputTelemetry(){
        Logger.recordOutput("Mechs", mechanismPoses);
  }

public static void updateIntakeAngle(double position) {
    mechanismPoses[0] = new Pose3d(new Translation3d(-.314, 0, .272), new Rotation3d(Units.degreesToRadians(0),
			Units.degreesToRadians(position), Units.degreesToRadians(0)));
}

public static void updatePivotHeight(double position) {
    Pose3d current = new Pose3d(Math.cos(Units.degreesToRadians(84)) * position, 0,
				Math.sin(Units.degreesToRadians(84)) * position, new Rotation3d());

		mechanismPoses[1] = current.div(3);
		mechanismPoses[2] = current.div(3).times(2);
		mechanismPoses[3] = current;
}

public static void updateEndEffectorAngle(double position) {
    mechanismPoses[4] = mechanismPoses[3]
				.transformBy(new Transform3d(new Translation3d(.22, 0, .2922), new Rotation3d(Units.degreesToRadians(0),
						Units.degreesToRadians(position+89), Units.degreesToRadians(0))));

}

}
