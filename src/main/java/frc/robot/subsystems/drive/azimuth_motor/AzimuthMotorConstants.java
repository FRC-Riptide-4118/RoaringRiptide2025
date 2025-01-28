package frc.robot.subsystems.drive.azimuth_motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.position_joint.PositionJointConstants.EncoderType;

public class AzimuthMotorConstants {
  public static final String canBusName = "";

  public record AzimuthMotorGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kMaxVelo,
      double kMaxAccel) {}

  public record AzimuthMotorHardwareConfig(
      int[] canIds,
      boolean[] reversed,
      double gearRatio,
      double currentLimit,
      EncoderType encoderType,
      int encoderID,
      Rotation2d encoderOffset,
      String canBus) {}

  public static final AzimuthMotorHardwareConfig FRONT_LEFT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {41},
          new boolean[] {true},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER,
          21,
          Rotation2d.fromRotations(0.387207),
          canBusName);

  public static final AzimuthMotorHardwareConfig FRONT_RIGHT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {42},
          new boolean[] {true},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER,
          22,
          Rotation2d.fromRotations(-0.050293),
          canBusName);

  public static final AzimuthMotorHardwareConfig BACK_LEFT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {43},
          new boolean[] {true},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER,
          23,
          Rotation2d.fromRotations(-0.439697),
          canBusName);

  public static final AzimuthMotorHardwareConfig BACK_RIGHT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {44},
          new boolean[] {true},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER,
          24,
          Rotation2d.fromRotations(0.479004),
          canBusName);

  public static final AzimuthMotorGains FRONT_LEFT_GAINS =
      new AzimuthMotorGains(3.0, 0.0, 0.0, 0.1, 3.2, 0.2, 4.0, 3.0);

  public static final AzimuthMotorGains FRONT_RIGHT_GAINS =
      new AzimuthMotorGains(3.0, 0.0, 0.0, 0.1, 3.2, 0.2, 4.0, 3.0);

  public static final AzimuthMotorGains BACK_LEFT_GAINS =
      new AzimuthMotorGains(3.0, 0.0, 0.0, 0.1, 3.2, 0.2, 4.0, 3.0);

  public static final AzimuthMotorGains BACK_RIGHT_GAINS =
      new AzimuthMotorGains(3.0, 0.0, 0.0, 0.1, 3.2, 0.2, 4.0, 3.0);
}
