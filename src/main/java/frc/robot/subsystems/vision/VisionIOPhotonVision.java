package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera m_camera;
  protected final Transform3d m_robotToCamera;

    /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    m_camera = new PhotonCamera(name);
    this.m_robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = m_camera.isConnected();

    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    // Read new camera observations

    for (var result : m_camera.getAllUnreadResults()) {
      // Find the best target
      PhotonTrackedTarget bestTarget = null;

      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          // Update latest target observation when a tracked target is visible;
          // leave the previous value unchanged when no tracked targets are detected
          if (!TRACKED_TAG_IDS.contains(target.fiducialId))
            continue;

          if (bestTarget == null || target.getPoseAmbiguity() < bestTarget.getPoseAmbiguity())
            bestTarget = target;
        }

        // Update latest target observation (yaw/pitch)
        if (bestTarget != null) {
          inputs.latestTargetObservation =
              new TargetObservation(
                  Rotation2d.fromDegrees(bestTarget.getYaw()),
                  Rotation2d.fromDegrees(bestTarget.getPitch()));
        }
      } // result.hasTargets()

      // Add pose observation
      if (result.multitagResult.isPresent()) {
        // Multi-tag pose estimation
        var multitagResult = result.multitagResult.get();

        // Skip multi-tag results that include any untracked tag IDs, since the PnP solve
        // already incorporated those tags and cannot be recomputed without them
        boolean allTagsTracked =
            multitagResult.fiducialIDsUsed.stream()
                .allMatch(id -> TRACKED_TAG_IDS.contains(id.intValue()));
        if (!allTagsTracked) continue;

        // Compute robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(m_robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Improved distance metric (min distance of used tags) in the multi-tag PnP solve
        double minTagDistance = Double.POSITIVE_INFINITY;

        for (short id : multitagResult.fiducialIDsUsed) {
          for (var target : result.targets) {
            if (target.fiducialId == id) {
              double dist = target.bestCameraToTarget.getTranslation().getNorm();
              minTagDistance = Math.min(minTagDistance, dist);
              break;
            }

          }
        }

        double distanceMetric = (minTagDistance != Double.POSITIVE_INFINITY) ? minTagDistance : 0.0;

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add the pose observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                distanceMetric, // Min tag distance
                PoseObservationType.PHOTONVISION)); // Observation type
      } // result.multitagResult.isPresent()

      else if (result.hasTargets()) {
        // Single tag result (aka NO multi-tag solve available but
        // any number of tags may be in the results individually so we
        // want to find the "best" one to use).  We want to use the
        // "best" one that we found earlier as our target of choice.

        PhotonTrackedTarget target = bestTarget;
        if (target == null) continue;

        // Skip single-tag results for untracked tag IDs
        if (!TRACKED_TAG_IDS.contains(target.fiducialId)) continue;

        // Calculate robot pose if our target AprilTag has a pose with it
        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isEmpty()) continue;

        Transform3d fieldToTarget =
            new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
        Transform3d cameraToTarget = target.bestCameraToTarget;
        Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
        Transform3d fieldToRobot = fieldToCamera.plus(m_robotToCamera.inverse());
        Pose3d robotPose =
            new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Add tag ID
        tagIds.add((short) target.fiducialId);

        // Add the pose observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                target.poseAmbiguity, // Ambiguity
                1, // Tag count
                cameraToTarget.getTranslation().getNorm(), // Average tag distance
                PoseObservationType.PHOTONVISION)); // Observation type
      } // (!result.targets.isEmpty()
    } // for (m_camera.getAllUnreadResults())

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
  }
}