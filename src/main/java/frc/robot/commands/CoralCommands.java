package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.commands.presets.CoralPreset;
import frc.robot.subsystems.position_joint.PositionJoint;

public class CoralCommands {
  public CoralCommands() {}

  public static Command CoralCommand(
      PositionJoint elevator, PositionJoint coralWrist, CoralPreset preset) {

    return new SequentialCommandGroup(
        new InstantCommand(() -> {}),
        new ParallelCommandGroup(
            new PositionJointPositionCommand(elevator, preset::getElevatorPosition),
            new PositionJointPositionCommand(
                coralWrist, () -> preset.getPivotRotation().getRotations())));
  }
}
