package frc.robot.subsystems.components;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import org.dyn4j.geometry.Rotation;
import org.littletonrobotics.junction.Logger;

public class Components extends SubsystemBase {
  private final PositionJoint elevator;
  private final PositionJoint coralWrist;
  private final PositionJoint climber;
  private final Flywheel algaeIntake;

  public Components(
      PositionJoint elevator,
      PositionJoint coralWrist,
      PositionJoint climber,
      Flywheel algaeIntake) {
    this.elevator = elevator;
    this.coralWrist = coralWrist;
    this.climber = climber;
    this.algaeIntake = algaeIntake;
  }

  @Override
  public void periodic() {
    Pose3d[] poses = new Pose3d[4];
    poses[0] =
        new Pose3d(
            new Translation3d(0, 0.222250, 0.178435),
            new Rotation3d(Rotation.of(climber.getPosition()).toRadians(), 0, 0));

    poses[1] = new Pose3d(new Translation3d(0, 0, elevator.getPosition() / 2.0), new Rotation3d());
    poses[2] = new Pose3d(new Translation3d(0, 0, elevator.getPosition()), new Rotation3d());
    poses[3] =
        new Pose3d(
            new Translation3d(0, -0.369025, 0.553517 + elevator.getPosition()),
            new Rotation3d(Rotation.of(coralWrist.getPosition()).toRadians(), 0, 0));

    Logger.recordOutput("Components", poses);
  }
}
