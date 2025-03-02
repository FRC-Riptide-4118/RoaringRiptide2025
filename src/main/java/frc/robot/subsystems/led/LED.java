package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
  private final String name;

  private final LEDIO led;
  private final LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged();

  private final LoggedTunableNumber kStatus;

  public LED(LEDIO io) {
    led = io;

    name = led.getName();

    kStatus = new LoggedTunableNumber(name + "/Status", 0);
  }

  @Override
  public void periodic() {
    led.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          led.setStatus(values[0]);
          System.out.println(("Changed LED values"));
        },
        kStatus);
  }

  public void setStatus(double status) {
    led.setStatus(status);
  }
}
