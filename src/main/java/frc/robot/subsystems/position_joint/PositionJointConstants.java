package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.geometry.Rotation2d;

public class PositionJointConstants {
  public enum GravityType {
    CONSTANT,
    COSINE,
    // Not supported by TalonFX
    SINE
  }

  public enum EncoderType {
    INTERNAL,
    EXTERNAL_CANCODER,
    EXTERNAL_CANCODER_PRO,
    EXTERNAL_DIO,
    EXTERNAL_SPARK
  }

  public record PositionJointGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kG,
      double kV,
      double kA,
      double kMaxVelo,
      double kMaxAccel,
      double kMinPosition,
      double kMaxPosition,
      double kTolerance,
      double kDefaultSetpoint) {}

  // Position Joint Gear Ratio should be multiplied by Math.PI * 2 for rotation joints to convert
  // from rotations to radians
  public record PositionJointHardwareConfig(
      int[] canIds,
      boolean[] reversed,
      double gearRatio,
      int currentLimit,
      GravityType gravity,
      EncoderType encoderType,
      int encoderID,
      Rotation2d encoderOffset,
      String canBus) {}

  public static final PositionJointGains ELEVATOR_GAINS =
      new PositionJointGains(0.8, 0.0, 0, 0.13, 0.7, 4.0, 0.0, 3.0, 3.0, 0.0, 1.3, 0.0, 0.0);

  public static final PositionJointGains ELEVATOR_GAINS_SIM =
      new PositionJointGains(0.0, 0.0, 0, 0.0, 0.0, 1.0, 0.0, 0.5, 0.5, 0.0, 1.0, 0.0, 0.0);
  // 36 / 1.27
  public static final PositionJointHardwareConfig ELEVATOR_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {19, 20},
          new boolean[] {true, true},
          36.0 / 1.27,
          50,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          -1,
          new Rotation2d(),
          "");

  public static final PositionJointGains WRIST_GAINS =
      new PositionJointGains(0.0, 0.0, 0, 0.0, 0.0, 0.0, 0, 3.0, 3.0, 0.0, 0.3, 0, 0.22);

  public static final PositionJointGains WRIST_GAINS_SIM =
      new PositionJointGains(0, 0, 0, 0, 0, 0.0, 0, 0.0, 0.0, 0.0, 1.0, 0, 0);

  public static final PositionJointHardwareConfig WRIST_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {11},
          new boolean[] {false},
          9.0 / 1.0,
          45,
          GravityType.COSINE,
          EncoderType.EXTERNAL_CANCODER,
          2,
          Rotation2d.fromRotations(-0.046875),
          // new Rotation2d(),
          "");

  public static final PositionJointGains CLIMBER_GAINS =
      new PositionJointGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  public static final PositionJointGains CLIMBER_GAINS_SIM =
      new PositionJointGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  public static final PositionJointHardwareConfig CLIMBER_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {21, 22},
          new boolean[] {false, true},
          36.0 / 1.0,
          60,
          GravityType.COSINE,
          EncoderType.INTERNAL,
          -1,
          Rotation2d.fromRotations(0),
          "");
}
