package frc.robot.subsystems.led;

public class LEDIOReplay implements LEDIO {

  private String name;

  public LEDIOReplay(String name) {
    this.name = name;
  }

  @Override
  public String getName() {
    return name;
  }
}
