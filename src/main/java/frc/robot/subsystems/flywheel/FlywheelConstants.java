package frc.robot.subsystems.flywheel;

public class FlywheelConstants {
  public record FlywheelGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kMaxAccel,
      double kTolerance) {}

  public record FlywheelHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, int currentLimit, String canBus) {}

  public static final FlywheelGains CORAL_INTAKE_GAINS =
      new FlywheelGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public static final FlywheelGains CORAL_INTAKE_GAINS_SIM =
      new FlywheelGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public static final FlywheelHardwareConfig CORAL_INTAKE_CONFIG =
      new FlywheelHardwareConfig(new int[] {10}, new boolean[] {false}, 25.0 / 1.0, 20, "");

  public static final FlywheelGains ALGAE_INTAKE_GAINS =
      new FlywheelGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public static final FlywheelGains ALGAE_INTAKE_GAINS_SIM =
      new FlywheelGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public static final FlywheelHardwareConfig ALGAE_INTAKE_CONFIG =
      new FlywheelHardwareConfig(
          new int[] {12, 13}, new boolean[] {false, true}, 20.0 / 1.0, 20, "");
}
