package frc.robot.commands.presets;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class DrivePreset {
  private final LoggedTunableNumber headingPreset;

  private final LoggedNetworkBoolean chooser;

  private final Trigger chooserTrigger;

  public DrivePreset(String presetName, Rotation2d headingPosition) {
    headingPreset =
        new LoggedTunableNumber("/DrivePresets/" + presetName, headingPosition.getDegrees());

    chooser = new LoggedNetworkBoolean("/DriveChoosers/" + presetName, false);

    chooserTrigger = new Trigger(chooser::get);
  }

  public Rotation2d getHeadingRotation() {
    return Rotation2d.fromDegrees(headingPreset.get());
  }

  public Trigger getChooserTrigger() {
    return chooserTrigger;
  }

  public void addCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    Command turnCommand =
        DriveCommands.joystickDriveAtAngleAssisted(
            drive, xSupplier, ySupplier, omegaSupplier, this::getHeadingRotation);

    // getChooserTrigger().onFalse(new InstantCommand(() -> {}, drive));
    getChooserTrigger().onTrue(turnCommand.handleInterrupt(() -> chooser.set(false)));
  }
}
