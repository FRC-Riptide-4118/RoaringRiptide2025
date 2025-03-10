package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

public class WristPreset {
  private final LoggedTunableNumber wristPos;

  public WristPreset(String name, Rotation2d wristPos) {
    this.wristPos = new LoggedTunableNumber("CoralPresets/" + name);
  }

  public Rotation2d getWristPos() {
    return Rotation2d.fromDegrees(wristPos.get());
  }
}
