package com.team5817.frc2025;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class RobotVisualizer {
    public static Pose3d[] mechanismPoses = new Pose3d[3];
    static {
    for (int i = 0; i < 3; i++) {
      mechanismPoses[i] = new Pose3d();
    }}

  public static void outputTelemetry(){
        Logger.recordOutput("Mechs", mechanismPoses);
  }



public static void updatePivotHeight(double position) {

    mechanismPoses[0] = new Pose3d(new Translation3d(-0.10578845427745062, 0, .14025192931185687), new Rotation3d(Units.degreesToRadians(0),
    Units.degreesToRadians(position), Units.degreesToRadians(0)));
}
//add speaker angle stuff


}
