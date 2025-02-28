package frc.robot.subsystems.drive.azimuth_motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.position_joint.PositionJointConstants.EncoderType;

public class AzimuthMotorConstants {
  public static final String canBusName = "";

  public record AzimuthMotorGains(
      double kP, double kI, double kD, double kS, double kV, double kA) {}

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
          Rotation2d.fromRotations(0.487305),
          canBusName);

  public static final AzimuthMotorHardwareConfig FRONT_RIGHT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {42},
          new boolean[] {true},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER,
          22,
          Rotation2d.fromRotations(0.289062),
          canBusName);

  public static final AzimuthMotorHardwareConfig BACK_LEFT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {43},
          new boolean[] {true},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER,
          23,
          Rotation2d.fromRotations(0.084717),
          canBusName);

  public static final AzimuthMotorHardwareConfig BACK_RIGHT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {44},
          new boolean[] {true},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER,
          24,
          Rotation2d.fromRotations(0.435791),
          canBusName);

  public static final AzimuthMotorGains AZIMUTH_MOTOR_GAINS =
      new AzimuthMotorGains(2.0, 0.0, 0.0, 0.1, 2.0, 0.0);

  public static final AzimuthMotorGains AZIMUTH_MOTOR_GAINS_SIM =
      new AzimuthMotorGains(3.0, 0.0, 0.0, 0.0, 2.5, 0.0);
}
