package frc.robot.subsystems.simulation;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class SimulationViewer extends SubsystemBase {
  private static final double kElevatorMinimumLength = 0.5;

  private final PWMSparkMax m_wristMotor = new PWMSparkMax(1);
  private final PWMSparkMax m_elevatorMotor = new PWMSparkMax(0);
  private final MechanismLigament2d m_elevator;
  private final MechanismLigament2d m_wrist;
  private final DoubleSupplier doubleSupplier;
  private final AnalogPotentiometer m_wristPot = new AnalogPotentiometer(1, 90);
  private final Joystick m_joystick = new Joystick(0);

  public SimulationViewer(DoubleSupplier doubleSupplier) {
    this.doubleSupplier = doubleSupplier;
    // the main mechanism object
    Mechanism2d mech = new Mechanism2d(3, 3);
    // the mechanism root node
    MechanismRoot2d root = mech.getRoot("climber", 2, 0);

    // MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
    // off the root node or another ligament object
    m_elevator = root.append(new MechanismLigament2d("elevator", kElevatorMinimumLength, 90));
    m_wrist =
        m_elevator.append(
            new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));

    // post the mechanism to the dashboard
    SmartDashboard.putData("Mech2d", mech);
  }

  @Override
  public void periodic() {
    // if there are bugs, change this to robot periodic'
    double elevatorPosition = doubleSupplier.getAsDouble();
    m_elevator.setLength(kElevatorMinimumLength + elevatorPosition);
    m_wrist.setAngle(m_wristPot.get());

    // if there are bugs, change this to teleop periodic
    m_elevatorMotor.set(m_joystick.getRawAxis(0));
    m_wristMotor.set(m_joystick.getRawAxis(1));
  }
}
