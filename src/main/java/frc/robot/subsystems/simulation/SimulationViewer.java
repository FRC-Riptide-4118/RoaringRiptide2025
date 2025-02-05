package frc.robot.subsystems.simulation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SimulationViewer extends SubsystemBase {
  private static final double kElevatorMinimumLength = 0.5;

  private final LoggedMechanismLigament2d m_elevator;
  // private final MechanismLigament2d m_wrist;
  private final DoubleSupplier elevatorPosSupplier;

  public SimulationViewer(DoubleSupplier doubleSupplier) {
    this.elevatorPosSupplier = doubleSupplier;
    // the main mechanism object
    LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
    // the mechanism root node
    LoggedMechanismRoot2d root = mech.getRoot("structure", 2, 0);

    // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
    // off the root node or another ligament object
    m_elevator = root.append(new LoggedMechanismLigament2d("elevator", kElevatorMinimumLength, 90));
    // m_wrist =
    //     m_elevator.append(
    //         new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));

    // post the mechanism to the dashboard
    Logger.recordOutput("elevator", mech);
  }

  @Override
  public void periodic() {
    // if there are bugs, change this to robot periodic
    double elevatorPosition = elevatorPosSupplier.getAsDouble();
    m_elevator.setLength(kElevatorMinimumLength + elevatorPosition);
    // m_wrist.setAngle(m_wristPot.get());

  }
}
