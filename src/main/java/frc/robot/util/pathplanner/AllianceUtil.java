package frc.robot.util.pathplanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceUtil {

  public static Rotation2d flipRotation2dAlliance(Rotation2d rotation) {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        return rotation;
      } else {
        // return Rotation2d.fromDegrees(180).minus(rotation);
        return rotation.plus(Rotation2d.fromDegrees(180));
      }
    } else {
      return rotation;
    }
  }
}
