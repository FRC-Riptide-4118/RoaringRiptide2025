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
      double currentLimit,
      GravityType gravity,
      EncoderType encoderType,
      int encoderID,
      Rotation2d encoderOffset,
      String canBus) {}

  public static final PositionJointGains ELEVATOR_GAINS =
      new PositionJointGains(0.05, 0.00001, 0, 0.112, 0, 0.115, 0.008, 20, 30, 0, 100, 0, 0);

  public static final PositionJointHardwareConfig ELEVATOR_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {19, 20},
          new boolean[] {false, true},
          1.0 / 1.0,
          15,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          -1,
          new Rotation2d(),
          "");

  public static final PositionJointGains WRIST_GAINS =
      new PositionJointGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  public static final PositionJointHardwareConfig WRIST_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {10},
          new boolean[] {false},
          // TODO: might have to change gear ratio
          25.0 / 1.0,
          40,
          GravityType.COSINE,
          EncoderType.EXTERNAL_CANCODER,
          1,
          Rotation2d.fromRotations(0),
          "");
}
