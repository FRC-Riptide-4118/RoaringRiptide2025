package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
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
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointConstants;
import frc.robot.subsystems.position_joint.PositionJointIOReplay;
import frc.robot.subsystems.position_joint.PositionJointIOSim;
import frc.robot.subsystems.position_joint.PositionJointIOSparkMax;
import frc.robot.subsystems.simulation.SimulationViewer;
import frc.robot.subsystems.vision.TrigVisionIOPhotonVision;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Flywheel coralIntake;

  private final Flywheel algaeIntake;

  private final PositionJoint elevator;

  private final PositionJoint wrist;

  private final SimulationViewer simulationViewer;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
                    DriveMotorConstants.FRONT_LEFT_GAINS,
                    new AzimuthMotorIOSparkMax(
                        "FrontLeftAz", AzimuthMotorConstants.FRONT_LEFT_CONFIG),
                    AzimuthMotorConstants.FRONT_LEFT_GAINS),
                new Module(
                    new DriveMotorIOSparkMax(
                        "FrontRightDrive", DriveMotorConstants.FRONT_RIGHT_CONFIG),
                    DriveMotorConstants.FRONT_RIGHT_GAINS,
                    new AzimuthMotorIOSparkMax(
                        "FrontRightAz", AzimuthMotorConstants.FRONT_RIGHT_CONFIG),
                    AzimuthMotorConstants.FRONT_RIGHT_GAINS),
                new Module(
                    new DriveMotorIOSparkMax("BackLeftDrive", DriveMotorConstants.BACK_LEFT_CONFIG),
                    DriveMotorConstants.BACK_LEFT_GAINS,
                    new AzimuthMotorIOSparkMax(
                        "BackLeftAz", AzimuthMotorConstants.BACK_LEFT_CONFIG),
                    AzimuthMotorConstants.BACK_LEFT_GAINS),
                new Module(
                    new DriveMotorIOSparkMax(
                        "BackRightDrive", DriveMotorConstants.BACK_RIGHT_CONFIG),
                    DriveMotorConstants.BACK_RIGHT_GAINS,
                    new AzimuthMotorIOSparkMax(
                        "BackRightAz", AzimuthMotorConstants.BACK_RIGHT_CONFIG),
                    AzimuthMotorConstants.BACK_RIGHT_GAINS),
                null,
                SparkOdometryThread.getInstance());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new TrigVisionIOPhotonVision(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    drive::getRotation));

        elevator =
            new PositionJoint(
                new PositionJointIOSparkMax("Elevator", PositionJointConstants.ELEVATOR_CONFIG),
                PositionJointConstants.ELEVATOR_GAINS);

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

        simulationViewer = new SimulationViewer(elevator::getPosition);

        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new Module(
                    new DriveMotorIOSim("FrontLeftDrive", DriveMotorConstants.FRONT_LEFT_CONFIG),
                    DriveMotorConstants.FRONT_LEFT_GAINS,
                    new AzimuthMotorIOSim("FrontLeftAz", AzimuthMotorConstants.FRONT_LEFT_CONFIG),
                    AzimuthMotorConstants.FRONT_LEFT_GAINS),
                new Module(
                    new DriveMotorIOSim("FrontRightDrive", DriveMotorConstants.FRONT_RIGHT_CONFIG),
                    DriveMotorConstants.FRONT_RIGHT_GAINS,
                    new AzimuthMotorIOSim("FrontRightAz", AzimuthMotorConstants.FRONT_RIGHT_CONFIG),
                    AzimuthMotorConstants.FRONT_RIGHT_GAINS),
                new Module(
                    new DriveMotorIOSim("BackLeftDrive", DriveMotorConstants.BACK_LEFT_CONFIG),
                    DriveMotorConstants.BACK_LEFT_GAINS,
                    new AzimuthMotorIOSim("BackLeftAz", AzimuthMotorConstants.BACK_LEFT_CONFIG),
                    AzimuthMotorConstants.BACK_LEFT_GAINS),
                new Module(
                    new DriveMotorIOSim("BackRightDrive", DriveMotorConstants.BACK_RIGHT_CONFIG),
                    DriveMotorConstants.BACK_RIGHT_GAINS,
                    new AzimuthMotorIOSim("BackRightAz", AzimuthMotorConstants.BACK_RIGHT_CONFIG),
                    AzimuthMotorConstants.BACK_RIGHT_GAINS),
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
                PositionJointConstants.ELEVATOR_GAINS);

        wrist =
            new PositionJoint(
                new PositionJointIOSim("Wrist", PositionJointConstants.WRIST_CONFIG),
                PositionJointConstants.WRIST_GAINS);

        coralIntake =
            new Flywheel(
                new FlywheelIOSim("Coral Intake", FlywheelConstants.CORAL_INTAKE_CONFIG),
                FlywheelConstants.CORAL_INTAKE_GAINS);

        algaeIntake =
            new Flywheel(
                new FlywheelIOSim("Algae Intake", FlywheelConstants.ALGAE_INTAKE_CONFIG),
                FlywheelConstants.ALGAE_INTAKE_GAINS);

        simulationViewer = new SimulationViewer(elevator::getPosition);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new Module(
                    new DriveMotorIOReplay("FrontLeftDrive"),
                    DriveMotorConstants.FRONT_LEFT_GAINS,
                    new AzimuthMotorIOReplay("FrontLeftAz"),
                    AzimuthMotorConstants.FRONT_LEFT_GAINS),
                new Module(
                    new DriveMotorIOReplay("FrontRightDrive"),
                    DriveMotorConstants.FRONT_RIGHT_GAINS,
                    new AzimuthMotorIOReplay("FrontRightAz"),
                    AzimuthMotorConstants.FRONT_RIGHT_GAINS),
                new Module(
                    new DriveMotorIOReplay("BackLeftDrive"),
                    DriveMotorConstants.BACK_LEFT_GAINS,
                    new AzimuthMotorIOReplay("BackLeftAz"),
                    AzimuthMotorConstants.BACK_LEFT_GAINS),
                new Module(
                    new DriveMotorIOReplay("BackRightDrive"),
                    DriveMotorConstants.BACK_RIGHT_GAINS,
                    new AzimuthMotorIOReplay("BackRightAz"),
                    AzimuthMotorConstants.BACK_RIGHT_GAINS),
                null,
                null);
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        elevator =
            new PositionJoint(
                new PositionJointIOReplay("Elevator"), PositionJointConstants.ELEVATOR_GAINS);

        wrist =
            new PositionJoint(
                new PositionJointIOReplay("Wrist"), PositionJointConstants.WRIST_GAINS);

        coralIntake =
            new Flywheel(
                new FlywheelIOReplay("Coral Intake"), FlywheelConstants.CORAL_INTAKE_GAINS);

        algaeIntake =
            new Flywheel(
                new FlywheelIOReplay("Algae Intake"), FlywheelConstants.ALGAE_INTAKE_GAINS);

        simulationViewer = new SimulationViewer(elevator::getPosition);
        break;
    }

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

    // Configure the button bindings
    configureButtonBindings();
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
            // vision,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    driverController.y().whileTrue(DriveCommands.driveToReef());
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

    // Reset gyro to 0° when B button is pressed
    driverController.b().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
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
