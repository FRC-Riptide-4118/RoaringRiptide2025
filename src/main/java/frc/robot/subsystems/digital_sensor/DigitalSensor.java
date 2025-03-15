package frc.robot.subsystems.digital_sensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class DigitalSensor extends SubsystemBase {
  private final DigitalSensorIO digitalSensor;
  private final DigitalSensorIOInputsAutoLogged inputs = new DigitalSensorIOInputsAutoLogged();

  private final LoggedTunableNumber kStatus;

  private final String name;

  public DigitalSensor(DigitalSensorIO io) {
    digitalSensor = io;

    name = digitalSensor.getName();

    kStatus = new LoggedTunableNumber(name + "/Status", 0.0);
  }

  @Override
  public void periodic() {
    digitalSensor.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public boolean sensorActive() {
    return inputs.sensorActive;
  }

  public Trigger getTrigger() {
    return new Trigger(this::sensorActive);
  }
}
