package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;

public class CoralPresets {
  public static final CoralPreset ZERO = new CoralPreset("Zero", 0.0, Rotation2d.fromDegrees(105));
  public static final CoralPreset L1 = new CoralPreset("L1", 0.025, Rotation2d.fromDegrees(40));
  public static final CoralPreset L2 = new CoralPreset("L2", 0.2, Rotation2d.fromDegrees(45));
  public static final CoralPreset L3 = new CoralPreset("L3", 0.63, Rotation2d.fromDegrees(45));
  public static final CoralPreset L4 = new CoralPreset("L4", 1.34, Rotation2d.fromDegrees(40));
  public static final CoralPreset FLICK =
      new CoralPreset("FLICK", 1.34, Rotation2d.fromDegrees(70));
  public static final CoralPreset HUMAN_PLAYER =
      new CoralPreset("HumanPlayer", 0.0, Rotation2d.fromDegrees(110));
  // public static final CoralPreset HUMAN_PLAYER =
  //     new CoralPreset("HumanPlayer", 0.3, Rotation2d.fromDegrees(60));

  public static final CoralPreset AlgaeL2_L3 =
      new CoralPreset("AlgaeL2_L3", 0.5, Rotation2d.fromDegrees(105));
  public static final CoralPreset AlgaeL3_L4 =
      new CoralPreset("AlgaeL3_L4", 0.9, Rotation2d.fromDegrees(105));
}
