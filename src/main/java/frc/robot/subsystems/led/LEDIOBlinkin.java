package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDIOBlinkin implements LEDIO {
  private Spark led;

  private String name;

  public LEDIOBlinkin(String name, int channel) {
    led = new Spark(channel);

    this.name = name;
  }

  public void setStatus(double code) {
    led.set(code);
  }

  @Override
  public String getName() {
    return name;
  }
}
