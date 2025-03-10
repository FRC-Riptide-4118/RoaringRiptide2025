package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.subsystems.position_joint.PositionJoint;
import java.util.function.DoubleSupplier;

public class CoralCommands {

  public static Command CoralPresetCommand(
      PositionJoint elevator,
      DoubleSupplier elevatorPos,
      PositionJoint wrist,
      DoubleSupplier wristPos) {

    return new InstantCommand(
            () -> {
              // if (elevator.getCurrentCommand() != null) {
              //   elevator.getCurrentCommand().cancel();
              // }

              // if (wrist.getCurrentCommand() != null) {
              //   wrist.getCurrentCommand().cancel();
              // }
            })
        .andThen(
            new ParallelCommandGroup(
                new PositionJointPositionCommand(wrist, wristPos),
                new PositionJointPositionCommand(elevator, elevatorPos)));
  }

  public static Command CoralPresetCommand(
      PositionJoint elevator, PositionJoint wrist, CoralPreset preset) {
    return CoralPresetCommand(
        elevator, preset::getElevatorPos, wrist, () -> preset.getWristPos().getRotations());
  }

  // public static Command MoveWristCommand(PositionJoint wrist, double wristPos) {
  //   // return MoveWristCommand(wrist, wristPos);
  //   return
  // }
}
