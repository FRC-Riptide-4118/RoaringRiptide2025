package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants;
import java.util.List;

public class AutoDrivePresets {
  public static final List<Waypoint> A_WAYPOINTS =
      PathPlannerPath.waypointsFromPoses(
          new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)), FieldConstants.Reef.centerFaces[0]);

  public static final PathConstraints CONSTRAINTS =
      new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

  //   public static final PathPlannerPath DRIVE_TO_A =
  //      PathPlannerPath.fromPathFile("LeftHPToA");

}
