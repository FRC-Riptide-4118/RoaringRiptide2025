package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
  @AutoLog
  public class LEDIOInputs {}

  public default void updateInputs(LEDIOInputs inputs) {}

  public default void setStatus(double code) {}

  public String getName();
}
