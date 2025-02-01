package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

/** IO implementation for real PhotonVision hardware. */
public class TrigVisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public TrigVisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    Set<Double> bestDistances = new HashSet<>();
    Set<Double> worstDistances = new HashSet<>();

    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      Transform3d bestFieldToCamera = result.getBestTarget().getBestCameraToTarget();
      // Transform3d bestFieldToRobot = bestFieldToCamera.plus(robotToCamera.inverse());

      Transform3d worstFieldToCamera = result.getBestTarget().getAlternateCameraToTarget();
      // Transform3d worstFieldToRobot = worstFieldToCamera.plus(robotToCamera.inverse());

      double bestDistance = bestFieldToCamera.getTranslation().getNorm();
      double worstDistance = worstFieldToCamera.getTranslation().getNorm();

      bestDistances.add(bestDistance);
      worstDistances.add(worstDistance);

      Transform3d fieldToCameraTrig =
          new Transform3d(
              new Translation3d(
                  bestDistance * Math.cos(Math.toRadians(result.getBestTarget().getYaw())),
                  bestDistance * Math.sin(Math.toRadians(result.getBestTarget().getYaw())),
                  bestDistance * Math.sin(Math.toRadians(result.getBestTarget().getPitch()))),
              new Rotation3d());

      Transform3d fieldToRobot = fieldToCameraTrig.plus(robotToCamera.inverse());
      Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

      tagIds.add(result.getBestTarget().fiducialId);

      poseObservations.add(
          new PoseObservation(
              result.getTimestampSeconds(),
              robotPose,
              result.getBestTarget().getPoseAmbiguity(),
              1,
              bestDistance,
              PoseObservationType.PHOTONVISION));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }

    inputs.bestDistance = new double[bestDistances.size()];
    int j = 0;
    for (double bestDistance : bestDistances) {
      inputs.bestDistance[j] = bestDistance;
    }

    inputs.worstDistance = new double[worstDistances.size()];
    int k = 0;
    for (double worstDistance : worstDistances) {
      inputs.worstDistance[k] = worstDistance;
    }
  }
}
