package frc.robot.commands.presets;

import edu.wpi.first.math.geometry.Rotation2d;

public class Presets {
  public static final CoralPreset L1 = new CoralPreset("L1", 0, Rotation2d.fromDegrees(0));
  public static final CoralPreset L2 = new CoralPreset("L2", 0.5, Rotation2d.fromDegrees(90));
  public static final CoralPreset L3 = new CoralPreset("L3", 0.5, Rotation2d.fromDegrees(90));
  public static final CoralPreset L4 = new CoralPreset("L4", 0.5, Rotation2d.fromDegrees(90));

  public static final DrivePreset A = new DrivePreset("A", new Rotation2d());
  public static final DrivePreset B = new DrivePreset("B", Rotation2d.fromDegrees(90));
}
