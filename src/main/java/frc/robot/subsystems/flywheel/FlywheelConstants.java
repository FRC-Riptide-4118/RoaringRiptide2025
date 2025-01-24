package frc.robot.subsystems.flywheel;

public class FlywheelConstants {
  public record FlywheelGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kTolerance) {}

  public record FlywheelHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, String canBus) {}

  public static final FlywheelHardwareConfig EXAMPLE_CONFIG =
      new FlywheelHardwareConfig(new int[] {1, 2}, new boolean[] {true, false}, 1.0, "");

  public static final FlywheelGains EXAMPLE_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0);

  public static final FlywheelHardwareConfig MY_FLYWHEEL_CONFIG =
      // this will likely need to be changed for new robot, this line below is set up for scrappy
      // oof
      new FlywheelHardwareConfig(new int[] {20, 21}, new boolean[] {true, false}, 60.0 / 1.0, "");

  public static final FlywheelGains MY_FLYWHEEL_GAINS = new FlywheelGains(0, 0, 0, 0, 0, 0, 0);
}
