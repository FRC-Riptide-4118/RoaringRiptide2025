package frc.robot.subsystems.digital_sensor;

public class DigitalSensorConstants {
  public record DigitalSensorConfig(int id, boolean invert) {}

  public static final DigitalSensorConfig BEAMBREAK_CONFIG = new DigitalSensorConfig(9, false);
}
