package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.CoralCommands;
import frc.robot.commands.CoralPresets;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.subsystems.components.Components;
import frc.robot.subsystems.digital_sensor.DigitalSensor;
import frc.robot.subsystems.digital_sensor.DigitalSensorConstants;
import frc.robot.subsystems.digital_sensor.DigitalSensorIODigitialInput;
import frc.robot.subsystems.digital_sensor.DigitalSensorIOReplay;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOReplay;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOSim;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOSparkMax;
import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOReplay;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOSim;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOSparkMax;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2SparkThread;
import frc.robot.subsystems.drive.odometry_threads.SparkOdometryThread;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIOReplay;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDIOBlinkin;
import frc.robot.subsystems.led.LEDIOReplay;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointConstants;
import frc.robot.subsystems.position_joint.PositionJointIOReplay;
import frc.robot.subsystems.position_joint.PositionJointIOSim;
import frc.robot.subsystems.position_joint.PositionJointIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOPhotonVisionTrig;
import frc.robot.util.pathplanner.AdvancedPPHolonomicDriveController;
import frc.robot.util.pathplanner.AllianceUtil;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final Flywheel coralIntake;

  private final Flywheel algaeIntake;

  private final LED led;

  private final DigitalSensor beamBreak;

  private final DigitalSensor climberLimitSwitch;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final PositionJoint elevator;

  private final PositionJoint climber;

  private final PositionJoint wrist;

  @SuppressWarnings("unused")
  private final Components simComponents;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandGenericHID operatorController = new CommandGenericHID(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedNetworkBoolean abChooser;
  private final LoggedNetworkBoolean cdChooser;
  private final LoggedNetworkBoolean efChooser;
  private final LoggedNetworkBoolean ghChooser;
  private final LoggedNetworkBoolean ijChooser;
  private final LoggedNetworkBoolean klChooser;

  private final LoggedNetworkBoolean leftChooser;
  private final LoggedNetworkBoolean rightChooser;

  private final LoggedNetworkBoolean l1Chooser;
  private final LoggedNetworkBoolean l2Chooser;
  private final LoggedNetworkBoolean l3Chooser;
  private final LoggedNetworkBoolean l4Chooser;

  private final LoggedNetworkString reefStateIndicator;
  private final LoggedNetworkString coralLevelIndicator;

  private final LoggedNetworkBoolean manualMode;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        // If using REV hardware, set up the Spark Odometry Thread, if using CTRE hardware, set up
        // the Phoenix Odometry Thread, if using a combination of the two, set up both
        drive =
            new Drive(
                new GyroIOPigeon2SparkThread(0),
                new Module(
                    new DriveMotorIOSparkMax(
                        "FrontLeftDrive", DriveMotorConstants.FRONT_LEFT_CONFIG),
                    new AzimuthMotorIOSparkMax(
                        "FrontLeftAz", AzimuthMotorConstants.FRONT_LEFT_CONFIG)),
                new Module(
                    new DriveMotorIOSparkMax(
                        "FrontRightDrive", DriveMotorConstants.FRONT_RIGHT_CONFIG),
                    new AzimuthMotorIOSparkMax(
                        "FrontRightAz", AzimuthMotorConstants.FRONT_RIGHT_CONFIG)),
                new Module(
                    new DriveMotorIOSparkMax("BackLeftDrive", DriveMotorConstants.BACK_LEFT_CONFIG),
                    new AzimuthMotorIOSparkMax(
                        "BackLeftAz", AzimuthMotorConstants.BACK_LEFT_CONFIG)),
                new Module(
                    new DriveMotorIOSparkMax(
                        "BackRightDrive", DriveMotorConstants.BACK_RIGHT_CONFIG),
                    new AzimuthMotorIOSparkMax(
                        "BackRightAz", AzimuthMotorConstants.BACK_RIGHT_CONFIG)),
                DriveMotorConstants.DRIVE_MOTOR_GAINS,
                AzimuthMotorConstants.AZIMUTH_MOTOR_GAINS,
                null,
                SparkOdometryThread.getInstance());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionTrig(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    drive::getRotation));

        elevator =
            new PositionJoint(
                new PositionJointIOSparkMax("Elevator", PositionJointConstants.ELEVATOR_CONFIG),
                PositionJointConstants.ELEVATOR_GAINS);

        climber =
            new PositionJoint(
                new PositionJointIOSparkMax("Climber", PositionJointConstants.CLIMBER_CONFIG),
                PositionJointConstants.CLIMBER_GAINS);

        wrist =
            new PositionJoint(
                new PositionJointIOSparkMax("Wrist", PositionJointConstants.WRIST_CONFIG),
                PositionJointConstants.WRIST_GAINS);

        coralIntake =
            new Flywheel(
                new FlywheelIOSparkMax("Coral Intake", FlywheelConstants.CORAL_INTAKE_CONFIG),
                FlywheelConstants.CORAL_INTAKE_GAINS);

        algaeIntake =
            new Flywheel(
                new FlywheelIOSparkMax("Algae Intake", FlywheelConstants.ALGAE_INTAKE_CONFIG),
                FlywheelConstants.ALGAE_INTAKE_GAINS);

        led = new LED(new LEDIOBlinkin("LED", 9));

        beamBreak =
            new DigitalSensor(
                new DigitalSensorIODigitialInput(
                    "Coral Wrist Beam Break", DigitalSensorConstants.BEAMBREAK_CONFIG));

        climberLimitSwitch =
            new DigitalSensor(
                new DigitalSensorIODigitialInput(
                    "Climber Limit Switch", DigitalSensorConstants.CLIMBER_LIMIT_SWITCH_CONFIG));

        // intakeSensor = new DigitalSensor(new DigitalSensorIO());
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new Module(
                    new DriveMotorIOSim("FrontLeftDrive", DriveMotorConstants.FRONT_LEFT_CONFIG),
                    new AzimuthMotorIOSim("FrontLeftAz", AzimuthMotorConstants.FRONT_LEFT_CONFIG)),
                new Module(
                    new DriveMotorIOSim("FrontRightDrive", DriveMotorConstants.FRONT_RIGHT_CONFIG),
                    new AzimuthMotorIOSim(
                        "FrontRightAz", AzimuthMotorConstants.FRONT_RIGHT_CONFIG)),
                new Module(
                    new DriveMotorIOSim("BackLeftDrive", DriveMotorConstants.BACK_LEFT_CONFIG),
                    new AzimuthMotorIOSim("BackLeftAz", AzimuthMotorConstants.BACK_LEFT_CONFIG)),
                new Module(
                    new DriveMotorIOSim("BackRightDrive", DriveMotorConstants.BACK_RIGHT_CONFIG),
                    new AzimuthMotorIOSim("BackRightAz", AzimuthMotorConstants.BACK_RIGHT_CONFIG)),
                DriveMotorConstants.DRIVE_MOTOR_GAINS_SIM,
                AzimuthMotorConstants.AZIMUTH_MOTOR_GAINS_SIM,
                null,
                null);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));

        elevator =
            new PositionJoint(
                new PositionJointIOSim("Elevator", PositionJointConstants.ELEVATOR_CONFIG),
                PositionJointConstants.ELEVATOR_GAINS_SIM);

        climber =
            new PositionJoint(
                new PositionJointIOSim("Climber", PositionJointConstants.CLIMBER_CONFIG),
                PositionJointConstants.CLIMBER_GAINS_SIM);

        wrist =
            new PositionJoint(
                new PositionJointIOSim("Wrist", PositionJointConstants.WRIST_CONFIG),
                PositionJointConstants.WRIST_GAINS_SIM);

        coralIntake =
            new Flywheel(
                new FlywheelIOSim("Coral Intake", FlywheelConstants.CORAL_INTAKE_CONFIG),
                FlywheelConstants.CORAL_INTAKE_GAINS_SIM);

        algaeIntake =
            new Flywheel(
                new FlywheelIOSim("Algae Intake", FlywheelConstants.ALGAE_INTAKE_CONFIG),
                FlywheelConstants.ALGAE_INTAKE_GAINS_SIM);

        led = new LED(new LEDIOReplay("LED"));

        beamBreak =
            new DigitalSensor(
                new DigitalSensorIODigitialInput(
                    "Coral Wrist Beam Break", DigitalSensorConstants.BEAMBREAK_CONFIG));

        climberLimitSwitch =
            new DigitalSensor(
                new DigitalSensorIODigitialInput(
                    "Climber Limit Switch", DigitalSensorConstants.CLIMBER_LIMIT_SWITCH_CONFIG));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new Module(
                    new DriveMotorIOReplay("FrontLeftDrive"),
                    new AzimuthMotorIOReplay("FrontLeftAz")),
                new Module(
                    new DriveMotorIOReplay("FrontRightDrive"),
                    new AzimuthMotorIOReplay("FrontRightAz")),
                new Module(
                    new DriveMotorIOReplay("BackLeftDrive"),
                    new AzimuthMotorIOReplay("BackLeftAz")),
                new Module(
                    new DriveMotorIOReplay("BackRightDrive"),
                    new AzimuthMotorIOReplay("BackRightAz")),
                DriveMotorConstants.DRIVE_MOTOR_GAINS,
                AzimuthMotorConstants.AZIMUTH_MOTOR_GAINS,
                null,
                null);
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        elevator =
            new PositionJoint(
                new PositionJointIOReplay("Elevator"), PositionJointConstants.ELEVATOR_GAINS);

        climber =
            new PositionJoint(
                new PositionJointIOReplay("Climber"), PositionJointConstants.CLIMBER_GAINS);

        wrist =
            new PositionJoint(
                new PositionJointIOReplay("Wrist"), PositionJointConstants.WRIST_GAINS);

        coralIntake =
            new Flywheel(
                new FlywheelIOReplay("Coral Intake"), FlywheelConstants.CORAL_INTAKE_GAINS);

        algaeIntake =
            new Flywheel(
                new FlywheelIOReplay("Algae Intake"), FlywheelConstants.ALGAE_INTAKE_GAINS);

        led = new LED(new LEDIOReplay("LED"));

        beamBreak = new DigitalSensor(new DigitalSensorIOReplay("Coral Wrist Beam Break"));

        climberLimitSwitch =
            new DigitalSensor(
                new DigitalSensorIODigitialInput(
                    "Climber Limit Switch", DigitalSensorConstants.CLIMBER_LIMIT_SWITCH_CONFIG));
        break;
    }

    simComponents = new Components(elevator, wrist, climber);

    registerNamedCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    abChooser = new LoggedNetworkBoolean("CoralChoosers/ABChooser", false);
    cdChooser = new LoggedNetworkBoolean("CoralChoosers/CDChooser", false);
    efChooser = new LoggedNetworkBoolean("CoralChoosers/EFChooser", false);
    ghChooser = new LoggedNetworkBoolean("CoralChoosers/GHChooser", false);
    ijChooser = new LoggedNetworkBoolean("CoralChoosers/IJChooser", false);
    klChooser = new LoggedNetworkBoolean("CoralChoosers/KLChooser", false);

    reefStateIndicator = new LoggedNetworkString("CoralChoosers/ReefStateIndicator", "None");
    coralLevelIndicator = new LoggedNetworkString("CoralChoosers/CoralLevelIndicator", "None");

    leftChooser = new LoggedNetworkBoolean("CoralChoosers/LeftChooser", false);
    rightChooser = new LoggedNetworkBoolean("CoralChoosers/RightChooser", false);

    l1Chooser = new LoggedNetworkBoolean("CoralChoosers/L1Chooser", false);
    l2Chooser = new LoggedNetworkBoolean("CoralChoosers/L2Chooser", false);
    l3Chooser = new LoggedNetworkBoolean("CoralChoosers/L3Chooser", false);
    l4Chooser = new LoggedNetworkBoolean("CoralChoosers/L4Chooser", false);

    manualMode = new LoggedNetworkBoolean("CoralChoosers/ManualMode", false);

    // Configure the button bindings

    configureButtonBindings();

    //     if (beamBreak.getTrigger().getAsBoolean()) {
    //       led.setStatus(0.07);
    //     } else {
    //       led.setStatus(0.0);
    //     }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Coral
    // Goes at -1 volt constantly to keep coral inside
    coralIntake.setDefaultCommand(
        new FlywheelVoltageCommand(
            coralIntake,
            () ->
                6.0
                    * (driverController.getLeftTriggerAxis()
                        - driverController.getRightTriggerAxis())));

    // coralIntake.setDefaultCommand(
    //     new FlywheelVoltageCommand(
    //         coralIntake,
    //         () ->
    //         beamBreak.getTrigger().getAsBoolean()
    //             ? 6.0 * (driverController.getLeftTriggerAxis() -
    // driverController.getRightTriggerAxis())
    //             : 6.0 * driverController.getLeftTriggerAxis()
    //     )
    // );

    // Algae
    algaeIntake.setDefaultCommand(
        new FlywheelVoltageCommand(
            algaeIntake,
            () ->
                driverController.leftBumper().getAsBoolean()
                    ? -6.0
                    : driverController.rightBumper().getAsBoolean() ? 6.0 : 0.0));

    led.setStatus(0.07);

    // Switch to X pattern when X button is pressed
    // driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Flick Wrist Up if Needed
    // driverController.start().onTrue(CoralCommands.MoveWristCommand(wrist, 60));

    operatorController
        .button(8)
        .and(climberLimitSwitch.getTrigger().negate())
        .whileTrue(Commands.run(() -> climber.setVoltage(6.0)))
        .onFalse(new InstantCommand(() -> climber.setVoltage(0.0)));

    operatorController
        .button(7)
        .and(climberLimitSwitch.getTrigger().negate())
        .whileTrue(Commands.run(() -> climber.setVoltage(-6.0)))
        .onFalse(new InstantCommand(() -> climber.setVoltage(0.0)));

    // driverController.y().whileTrue(DriveCommands.driveToReef());
    // // Reset gyro / odometry
    final Runnable resetGyro =
        () ->
            drive.setPose(
                new Pose2d(
                    drive.getPose().getTranslation(),
                    DriverStation.getAlliance().isPresent()
                        ? (DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                            ? new Rotation2d(Math.PI)
                            : new Rotation2d())
                        : new Rotation2d())); // zero gyro

    // Reset gyro to 0°
    driverController.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    driverController
        .a()
        .or(operatorController.povDown())
        .onTrue(
            DriveCommands.joystickDriveAtAngleCancellable(
                drive,
                vision,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> AllianceUtil.flipRotation2dAlliance(Rotation2d.fromDegrees(180)),
                driverController.povUp()));

    driverController
        .a()
        .and(driverController.b())
        .or(operatorController.povDownRight())
        .onTrue(
            DriveCommands.joystickDriveAtAngleCancellable(
                drive,
                vision,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> AllianceUtil.flipRotation2dAlliance(Rotation2d.fromDegrees(-120)),
                driverController.povUp()));

    driverController
        .b()
        .or(operatorController.povRight())
        .onTrue(
            DriveCommands.joystickDriveAtAngleCancellable(
                drive,
                vision,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> AllianceUtil.flipRotation2dAlliance(Rotation2d.fromDegrees(270)),
                driverController.povUp()));

    driverController
        .b()
        .and(driverController.y())
        .or(operatorController.povUpRight())
        .onTrue(
            DriveCommands.joystickDriveAtAngleCancellable(
                drive,
                vision,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> AllianceUtil.flipRotation2dAlliance(Rotation2d.fromDegrees(-60)),
                driverController.povUp()));

    driverController
        .y()
        .or(operatorController.povUp())
        .onTrue(
            DriveCommands.joystickDriveAtAngleCancellable(
                drive,
                vision,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> AllianceUtil.flipRotation2dAlliance(Rotation2d.fromDegrees(0)),
                driverController.povUp()));

    driverController
        .y()
        .and(driverController.x())
        .or(operatorController.povUpLeft())
        .onTrue(
            DriveCommands.joystickDriveAtAngleCancellable(
                drive,
                vision,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> AllianceUtil.flipRotation2dAlliance(Rotation2d.fromDegrees(60)),
                driverController.povUp()));

    driverController
        .x()
        .or(operatorController.povLeft())
        .onTrue(
            DriveCommands.joystickDriveAtAngleCancellable(
                drive,
                vision,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> AllianceUtil.flipRotation2dAlliance(Rotation2d.fromDegrees(90)),
                driverController.povUp()));

    driverController
        .x()
        .and(driverController.a())
        .or(operatorController.povDownLeft())
        .onTrue(
            DriveCommands.joystickDriveAtAngleCancellable(
                drive,
                vision,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> AllianceUtil.flipRotation2dAlliance(Rotation2d.fromDegrees(120)),
                driverController.povUp()));

    operatorController.povDown().onTrue(new InstantCommand(() -> abChooser.set(true)));
    operatorController.povDownRight().onTrue(new InstantCommand(() -> cdChooser.set(true)));
    operatorController.povUpRight().onTrue(new InstantCommand(() -> efChooser.set(true)));
    operatorController.povUp().onTrue(new InstantCommand(() -> ghChooser.set(true)));
    operatorController.povUpLeft().onTrue(new InstantCommand(() -> ijChooser.set(true)));
    operatorController.povDownLeft().onTrue(new InstantCommand(() -> klChooser.set(true)));

    operatorController
        .button(5)
        .onTrue(
            new InstantCommand(
                () -> {
                  l1Chooser.set(true);
                  l2Chooser.set(false);
                  l3Chooser.set(false);
                  l4Chooser.set(false);

                  //   if (elevator.getCurrentCommand() != null) {
                  //     elevator.getCurrentCommand().cancel();
                  //   }

                  //   if (wrist.getCurrentCommand() != null) {
                  //     wrist.getCurrentCommand().cancel();
                  //   }
                }))
        .onTrue(new InstantCommand(() -> coralLevelIndicator.set("L1")));

    operatorController
        .button(3)
        .onTrue(
            new InstantCommand(
                () -> {
                  l1Chooser.set(false);
                  l2Chooser.set(true);
                  l3Chooser.set(false);
                  l4Chooser.set(false);

                  //   if (elevator.getCurrentCommand() != null) {
                  //     elevator.getCurrentCommand().cancel();
                  //   }

                  //   if (wrist.getCurrentCommand() != null) {
                  //     wrist.getCurrentCommand().cancel();
                  //   }
                }))
        .onTrue(new InstantCommand(() -> coralLevelIndicator.set("L2")));

    operatorController
        .button(4)
        .onTrue(
            new InstantCommand(
                () -> {
                  l1Chooser.set(false);
                  l2Chooser.set(false);
                  l3Chooser.set(true);
                  l4Chooser.set(false);

                  //   if (elevator.getCurrentCommand() != null) {
                  //     elevator.getCurrentCommand().cancel();
                  //   }

                  //   if (wrist.getCurrentCommand() != null) {
                  //     wrist.getCurrentCommand().cancel();
                  //   }
                }))
        .onTrue(new InstantCommand(() -> coralLevelIndicator.set("L3")));

    operatorController
        .button(6)
        .onTrue(
            new InstantCommand(
                () -> {
                  l1Chooser.set(false);
                  l2Chooser.set(false);
                  l3Chooser.set(false);
                  l4Chooser.set(true);

                  //   if (elevator.getCurrentCommand() != null) {
                  //     elevator.getCurrentCommand().cancel();
                  //   }

                  //   if (wrist.getCurrentCommand() != null) {
                  //     wrist.getCurrentCommand().cancel();
                  //   }
                }))
        .onTrue(new InstantCommand(() -> coralLevelIndicator.set("L4")));

    operatorController
        .button(1)
        .onTrue(new InstantCommand(() -> coralLevelIndicator.set("AlgaeL2_L3")));

    operatorController
        .button(2)
        .onTrue(new InstantCommand(() -> coralLevelIndicator.set("AlgaeL3_L4")));

    operatorController
        .button(10)
        .onTrue(new InstantCommand(() -> manualMode.set(true)))
        .onFalse(new InstantCommand(() -> manualMode.set(false)));

    operatorController.button(9).onTrue(new InstantCommand(() -> coralLevelIndicator.set("Zero")));

    // Pick up from Human Player
    new Trigger(() -> operatorController.getRawAxis(2) > 0.5)
        .onTrue(new InstantCommand(() -> coralLevelIndicator.set("Human Player")));

    // Flick on L4 to Score
    new Trigger(() -> operatorController.getRawAxis(3) > 0.5)
        .onTrue(new InstantCommand(() -> coralLevelIndicator.set("FLICK")))
        .onTrue(new InstantCommand(() -> coralIntake.setVoltage(-6), coralIntake));

    new Trigger(abChooser::get)
        .onTrue(
            new InstantCommand(
                () -> {
                  cdChooser.set(false);
                  efChooser.set(false);
                  ghChooser.set(false);
                  ijChooser.set(false);
                  klChooser.set(false);
                }))
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(() -> reefStateIndicator.set("A")),
                new InstantCommand(() -> reefStateIndicator.set("B")),
                leftChooser::get));

    new Trigger(cdChooser::get)
        .onTrue(
            new InstantCommand(
                () -> {
                  abChooser.set(false);
                  efChooser.set(false);
                  ghChooser.set(false);
                  ijChooser.set(false);
                  klChooser.set(false);
                }))
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(() -> reefStateIndicator.set("C")),
                new InstantCommand(() -> reefStateIndicator.set("D")),
                leftChooser::get));

    new Trigger(efChooser::get)
        .onTrue(
            new InstantCommand(
                () -> {
                  abChooser.set(false);
                  cdChooser.set(false);
                  ghChooser.set(false);
                  ijChooser.set(false);
                  klChooser.set(false);
                }))
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(() -> reefStateIndicator.set("E")),
                new InstantCommand(() -> reefStateIndicator.set("F")),
                leftChooser::get));

    new Trigger(ghChooser::get)
        .onTrue(
            new InstantCommand(
                () -> {
                  abChooser.set(false);
                  cdChooser.set(false);
                  efChooser.set(false);
                  ijChooser.set(false);
                  klChooser.set(false);
                }))
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(() -> reefStateIndicator.set("G")),
                new InstantCommand(() -> reefStateIndicator.set("H")),
                leftChooser::get));

    new Trigger(ijChooser::get)
        .onTrue(
            new InstantCommand(
                () -> {
                  abChooser.set(false);
                  cdChooser.set(false);
                  efChooser.set(false);
                  ghChooser.set(false);
                  klChooser.set(false);
                }))
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(() -> reefStateIndicator.set("I")),
                new InstantCommand(() -> reefStateIndicator.set("J")),
                leftChooser::get));

    new Trigger(klChooser::get)
        .onTrue(
            new InstantCommand(
                () -> {
                  abChooser.set(false);
                  cdChooser.set(false);
                  efChooser.set(false);
                  ghChooser.set(false);
                  ijChooser.set(false);
                }))
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(() -> reefStateIndicator.set("K")),
                new InstantCommand(() -> reefStateIndicator.set("L")),
                leftChooser::get));

    // operatorController
    //     .button(1)
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               leftChooser.set(true);
    //               rightChooser.set(false);
    //             }));

    // operatorController
    //     .button(2)
    //     .onTrue(
    //         new InstantCommand(
    //             () -> {
    //               leftChooser.set(false);
    //               rightChooser.set(true);
    //             }));

    /*
    // new Trigger(abChooser::get)
    //     .onTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> AllianceUtil.flipRotation2dAlliance(Rotation2d.fromDegrees(0))))
    //     .and(driverController.a())
    //     .whileTrue(
    //         new ConditionalCommand(
    //             DriveCommands.driveToReef(drive, "A"),
    //             DriveCommands.driveToReef(drive, "B"),
    //             leftChooser::get));

    // new Trigger(cdChooser::get)
    //     .onTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> AllianceUtil.flipRotation2dAlliance(Rotation2d.fromDegrees(60))))
    //     .and(driverController.a())
    //     .whileTrue(
    //         new ConditionalCommand(
    //             DriveCommands.driveToReef(drive, "C"),
    //             DriveCommands.driveToReef(drive, "D"),
    //             leftChooser::get));

    // new Trigger(efChooser::get)
    //     .onTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> AllianceUtil.flipRotation2dAlliance(Rotation2d.fromDegrees(120))))
    //     .and(driverController.a())
    //     .whileTrue(
    //         new ConditionalCommand(
    //             DriveCommands.driveToReef(drive, "E"),
    //             DriveCommands.driveToReef(drive, "F"),
    //             leftChooser::get));

    // new Trigger(ghChooser::get)
    //     .onTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> AllianceUtil.flipRotation2dAlliance(Rotation2d.fromDegrees(180))))
    //     .and(driverController.a())
    //     .whileTrue(
    //         new ConditionalCommand(
    //             DriveCommands.driveToReef(drive, "G"),
    //             DriveCommands.driveToReef(drive, "H"),
    //             leftChooser::get));

    // new Trigger(ijChooser::get)
    //     .onTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> AllianceUtil.flipRotation2dAlliance(Rotation2d.fromDegrees(-120))))
    //     .and(driverController.a())
    //     .whileTrue(
    //         new ConditionalCommand(
    //             DriveCommands.driveToReef(drive, "I"),
    //             DriveCommands.driveToReef(drive, "J"),
    //             leftChooser::get));

    // new Trigger(klChooser::get)
    //     .onTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> AllianceUtil.flipRotation2dAlliance(Rotation2d.fromDegrees(-60))))
    //     .and(driverController.a())
    //     .whileTrue(
    //         new ConditionalCommand(
    //             DriveCommands.driveToReef(drive, "K"),
    //             DriveCommands.driveToReef(drive, "L"),
    //             leftChooser::get));

    */
    new Trigger(manualMode::get)
        .whileTrue(
            new SelectCommand<String>(
                Map.of(
                    "L1",
                    CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.L1),
                    "L2",
                    CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.L2),
                    "L3",
                    CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.L3),
                    "L4",
                    CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.L4),
                    "Human Player",
                    CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.HUMAN_PLAYER),
                    "Zero",
                    CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.ZERO),
                    "FLICK",
                    CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.FLICK),
                    "AlgaeL2_L3",
                    CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.AlgaeL2_L3),
                    "AlgaeL3_L4",
                    CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.AlgaeL3_L4)),
                coralLevelIndicator::get));
  }

  public void registerNamedCommands() {
    // NamedCommands.registerCommand(
    //     "CoralPreset",
    //     new SelectCommand<String>(
    //         Map.of(
    //             "L1",
    //             CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.L1),
    //             "L2",
    //             CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.L2),
    //             "L3",
    //             CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.L3),
    //             "L4",
    //             CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.L4)),
    //         coralLevelIndicator::get));

    NamedCommands.registerCommand(
        "L4",
        CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.L4)
            .withName("L4")
            .withTimeout(1.5));

    NamedCommands.registerCommand(
        "L3",
        CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.L3)
            .withName("L3")
            .withTimeout(1.5));

    NamedCommands.registerCommand(
        "L2",
        CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.L2)
            .withName("L2")
            .withTimeout(1.5));

    NamedCommands.registerCommand(
        "L1",
        CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.L1)
            .withName("L1")
            .withTimeout(1.5));

    NamedCommands.registerCommand(
        "Zero",
        CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.ZERO)
            .withName("Zero")
            .withTimeout(0.7));

    NamedCommands.registerCommand(
        "Human Player",
        CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.HUMAN_PLAYER)
            .withName("Human Player")
            .withTimeout(0.7));

    NamedCommands.registerCommand(
        "Intake Coral",
        CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.HUMAN_PLAYER)
            .alongWith(
                new FlywheelVoltageCommand(coralIntake, () -> 6.0)
                    .withName("Intake Coral")
                    .withTimeout(1.5)
                    .andThen(
                        new FlywheelVoltageCommand(coralIntake, () -> 0.0)
                            .withName("Stop Coral Intake")
                            .withTimeout(0.0))));

    NamedCommands.registerCommand(
        "Outtake Coral",
        new FlywheelVoltageCommand(coralIntake, () -> 6.0)
            .withName("Outtake")
            .withTimeout(1)
            .andThen(
                new FlywheelVoltageCommand(coralIntake, () -> 0.0)
                    .withName("Stop Coral Intake")
                    .withTimeout(0.0)));

    NamedCommands.registerCommand(
        "Flick",
        new FlywheelVoltageCommand(coralIntake, () -> 6.0)
            .withName("Outtake")
            .alongWith(CoralCommands.CoralPresetCommand(elevator, wrist, CoralPresets.FLICK))
            .withName("Flick")
            .withTimeout(0.5));

    NamedCommands.registerCommand(
        "AlignToReef",
        new InstantCommand(() -> AdvancedPPHolonomicDriveController.overrideYFeedback(null)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
