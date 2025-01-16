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
      double currentLimit,
      GravityType gravity,
      EncoderType encoderType,
      int encoderID,
      Rotation2d encoderOffset,
      String canBus) {}

  public static final PositionJointGains ELEVATOR_GAINS =
      new PositionJointGains(
          20.0, 0.0, 0.0, 0.5, 1.0, 2.0, 0.0, 10.0, 20.0, 0.0, Math.PI, 0.2, 0.0);

  public static final PositionJointHardwareConfig ELEVATOR_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {10, 11},
          new boolean[] {false, false},
          80.0,
          40,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          -1,
          Rotation2d.fromRotations(0.5),
          "");

  public static final PositionJointGains CORAL_WRIST_GAINS =
      new PositionJointGains(
          10.0, 0.0, 0.0, 0.5, 1.0, 2.0, 0.0, 10.0, 20.0, 0.0, Math.PI, 0.2, 0.0);

  public static final PositionJointHardwareConfig CORAL_WRIST_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {12},
          new boolean[] {false},
          20.0,
          40,
          GravityType.COSINE,
          EncoderType.EXTERNAL_CANCODER,
          13,
          Rotation2d.fromRotations(0.5),
          "");

  public static final PositionJointGains CLIMBER_GAINS =
      new PositionJointGains(
          10.0, 0.0, 0.0, 0.5, 1.0, 2.0, 0.0, 10.0, 20.0, 0.0, Math.PI, 0.2, 0.0);

  public static final PositionJointHardwareConfig CLIMBER_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {14},
          new boolean[] {false},
          80,
          40,
          GravityType.COSINE,
          EncoderType.INTERNAL,
          -1,
          Rotation2d.fromRotations(0.5),
          "");
}
