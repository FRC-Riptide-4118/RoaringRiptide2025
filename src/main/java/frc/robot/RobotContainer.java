package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.subsystems.digital_sensor.DigitalSensor;
import frc.robot.subsystems.digital_sensor.DigitalSensorConstants;
import frc.robot.subsystems.digital_sensor.DigitalSensorIO;
import frc.robot.subsystems.digital_sensor.DigitalSensorIODigitialInput;
import frc.robot.subsystems.piece_detection.PieceDetection;
import frc.robot.subsystems.piece_detection.PieceDetectionConstants;
import frc.robot.subsystems.piece_detection.PieceDetectionIO;
import frc.robot.subsystems.piece_detection.PieceDetectionIOPhoton;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointConstants;
import frc.robot.subsystems.position_joint.PositionJointIO;
import frc.robot.subsystems.position_joint.PositionJointIONeo;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  //   private final Drive drive;

  @SuppressWarnings("unused")
  //   private final Vision vision;

  private final DigitalSensor beambreak;

  private final DigitalSensor digitalsensor;

  private final PositionJoint scrappyPositionJoint;

  private final PieceDetection scrappyPieceDetection;
  // Simulation
  //   private SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Dashboard inputs
  //   private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // drive =
        //     new Drive(
        //         new GyroIOPigeon2(0),
        //         new ModuleIOTalonFX(TalonFXModuleConstants.frontLeft),
        //         new ModuleIOTalonFX(TalonFXModuleConstants.frontRight),
        //         new ModuleIOTalonFX(TalonFXModuleConstants.rearLeft),
        //         new ModuleIOTalonFX(TalonFXModuleConstants.rearRight),
        //         PhoenixOdometryThread.getInstance());
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVision(
        //             VisionConstants.camera0Name, VisionConstants.robotToCamera0));
        digitalsensor =
            new DigitalSensor(
                new DigitalSensorIODigitialInput("Switch", DigitalSensorConstants.SWITCH_CONFIG));

        beambreak =
            new DigitalSensor(
                new DigitalSensorIODigitialInput(
                    "Beam break", DigitalSensorConstants.BEAMBREAK_CONFIG));

        scrappyPositionJoint =
            new PositionJoint(
                new PositionJointIONeo(
                    "Scrappy Position Joint", PositionJointConstants.SCRAPPY_POSITION_JOINT_CONFIG),
                PositionJointConstants.SCRAPPY_POSITION_JOINT_GAINS);

        scrappyPieceDetection =
            new PieceDetection(
                new PieceDetectionIOPhoton(
                    "Scrappy Piece Detection", PieceDetectionConstants.config));
        break;

      case SIM:
        // // create a maple-sim swerve drive simulation instance
        // driveSimulation =
        //     new SwerveDriveSimulation(
        //         DriveConstants.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        // // add the simulated drivetrain to the simulation field
        // SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        // drive =
        //     new Drive(
        //         new GyroIOSim(driveSimulation.getGyroSimulation()),
        //         new ModuleIOSparkSim(driveSimulation.getModules()[0]),
        //         new ModuleIOSparkSim(driveSimulation.getModules()[1]),
        //         new ModuleIOSparkSim(driveSimulation.getModules()[2]),
        //         new ModuleIOSparkSim(driveSimulation.getModules()[3]),
        //         null);

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.camera0Name,
        //             VisionConstants.robotToCamera0,
        //             driveSimulation::getSimulatedDriveTrainPose),
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.camera1Name,
        //             VisionConstants.robotToCamera1,
        //             driveSimulation::getSimulatedDriveTrainPose));

        digitalsensor = new DigitalSensor(new DigitalSensorIO() {});

        beambreak = new DigitalSensor(new DigitalSensorIO() {});

        scrappyPositionJoint =
            new PositionJoint(
                new PositionJointIO() {}, PositionJointConstants.SCRAPPY_POSITION_JOINT_GAINS);

        scrappyPieceDetection = new PieceDetection(new PieceDetectionIO() {});

        break;

      default:
        // Replayed robot, disable IO implementations
        // drive =
        //     new Drive(
        //         new GyroIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         new ModuleIO() {},
        //         null);
        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        digitalsensor = new DigitalSensor(new DigitalSensorIO() {});
        beambreak = new DigitalSensor(new DigitalSensorIO() {});
        scrappyPositionJoint =
            new PositionJoint(
                new PositionJointIO() {}, PositionJointConstants.SCRAPPY_POSITION_JOINT_GAINS);

        scrappyPieceDetection = new PieceDetection(new PieceDetectionIO() {});

        break;
    }

    // Set up auto routines
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -driverController.getLeftY(),
    //         () -> -driverController.getLeftX(),
    //         () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    // driverController
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // // Reset gyro / odometry
    // final Runnable resetGyro =
    //     Constants.currentMode == Constants.Mode.SIM
    //         ? () ->
    // drive.setPose(
    // driveSimulation
    // .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
    // simulation
    // : () ->
    //     drive.setPose(
    //         new Pose2d(
    //             drive.getPose().getTranslation(),
    //             DriverStation.getAlliance().isPresent()
    //                 ? (DriverStation.getAlliance().get() == DriverStation.Alliance.Red
    //                     ? new Rotation2d(Math.PI)
    //                     : new Rotation2d())
    //                 : new Rotation2d())); // zero gyro

    // Reset gyro to 0° when B button is pressed
    // driverController.b().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    beambreak.getTrigger().onTrue(new PrintCommand("true"));

    // spinny
    driverController
        .b()
        .and(driverController.a())
        .onTrue(new PositionJointPositionCommand(scrappyPositionJoint, () -> 100))
        .onFalse(new PositionJointPositionCommand(scrappyPositionJoint, () -> 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.get();
    return Commands.none();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    // driveSimulation.setSimulationWorldPose(drive.getPose());
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    // Logger.recordOutput(
    //     "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral",
        SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(new Pose3d[0]));
  }
}
