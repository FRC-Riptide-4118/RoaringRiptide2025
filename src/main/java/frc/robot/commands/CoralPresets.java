package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;

public class CoralPresets {
  public static final CoralPreset ZERO = new CoralPreset("Zero", 0, Rotation2d.fromDegrees(0));
  public static final CoralPreset L1 = new CoralPreset("L1", 0.3, Rotation2d.fromDegrees(0));
  public static final CoralPreset L2 = new CoralPreset("L2", 0.5, Rotation2d.fromDegrees(0));
  public static final CoralPreset L3 = new CoralPreset("L3", 0.8, Rotation2d.fromDegrees(0));
  public static final CoralPreset L4 = new CoralPreset("L4", 1, Rotation2d.fromDegrees(0));
}
