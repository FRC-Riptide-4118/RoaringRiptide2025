package frc.robot.commands.presets;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CoralCommands;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class CoralPreset {
  private final LoggedTunableNumber elevatorPreset;
  private final LoggedTunableNumber pivotPreset;

  private final LoggedNetworkBoolean chooser;

  private final Trigger chooserTrigger;

  public CoralPreset(String presetName, double elevatorPosition, Rotation2d pivotRotation) {
    elevatorPreset =
        new LoggedTunableNumber("/CoralPresets/" + presetName + "/elevator", elevatorPosition);
    pivotPreset =
        new LoggedTunableNumber(
            "/CoralPresets/" + presetName + "/pivot", pivotRotation.getDegrees());

    chooser = new LoggedNetworkBoolean("/CoralChoosers/" + presetName, false);

    chooserTrigger = new Trigger(chooser::get);
  }

  public double getElevatorPosition() {
    return elevatorPreset.get();
  }

  public Rotation2d getPivotRotation() {
    return Rotation2d.fromDegrees(pivotPreset.get());
  }

  public Trigger getChooserTrigger() {
    return chooserTrigger;
  }

  public void addCommand(PositionJoint elevator, PositionJoint coralWrist) {
    getChooserTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  if (elevator.getCurrentCommand() != null) {
                    elevator.getCurrentCommand().cancel();
                  }

                  if (coralWrist.getCurrentCommand() != null) {
                    coralWrist.getCurrentCommand().cancel();
                  }
                }));
    getChooserTrigger()
        .onTrue(
            CoralCommands.CoralCommand(elevator, coralWrist, this)
                .andThen(new InstantCommand(() -> chooser.set(false))));
  }
}
