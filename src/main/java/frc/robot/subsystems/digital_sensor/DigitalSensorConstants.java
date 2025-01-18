package frc.robot.subsystems.digital_sensor;

public class DigitalSensorConstants {
  public record DigitalSensorConfig(int id, boolean invert) {}

  public static final DigitalSensorConfig SWITCH_CONFIG = new DigitalSensorConfig(0, false);

  public static final DigitalSensorConfig BEAMBREAK_CONFIG = new DigitalSensorConfig(5, false);
}
