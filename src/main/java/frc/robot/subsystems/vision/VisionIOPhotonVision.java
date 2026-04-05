// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

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

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : m_camera.getAllUnreadResults()) {
      // Update latest target observation when a tracked target is visible;
      // leave the previous value unchanged when no tracked targets are detected
      if (result.hasTargets()) {
        for (var target : result.getTargets()) {
          if (TRACKED_TAG_IDS.contains(target.fiducialId)) {
            inputs.latestTargetObservation =
                new TargetObservation(
                    Rotation2d.fromDegrees(target.getYaw()),
                    Rotation2d.fromDegrees(target.getPitch()));
            break;
          }
        }
      }

      // Add pose observation
      if (result.multitagResult.isPresent()) { // Multitag result
        var multitagResult = result.multitagResult.get();

        // Skip multi-tag results that include any untracked tag IDs, since the PnP solve
        // already incorporated those tags and cannot be recomputed without them
        boolean allTagsTracked =
            multitagResult.fiducialIDsUsed.stream()
                .allMatch(id -> TRACKED_TAG_IDS.contains(id.intValue()));
        if (!allTagsTracked) continue;

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(m_robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance over only the tags used in the multi-tag PnP solve
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                totalTagDistance / multitagResult.fiducialIDsUsed.size(), // Average tag distance
                PoseObservationType.PHOTONVISION)); // Observation type

      } else if (!result.targets.isEmpty()) { // Single tag result
        var target = result.targets.get(0);

        // Skip single-tag results for untracked tag IDs
        if (!TRACKED_TAG_IDS.contains(target.fiducialId)) continue;

        // Calculate robot pose
        var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
        if (tagPose.isPresent()) {
          Transform3d fieldToTarget =
              new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
          Transform3d cameraToTarget = target.bestCameraToTarget;
          Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
          Transform3d fieldToRobot = fieldToCamera.plus(m_robotToCamera.inverse());
          Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

          // Add tag ID
          tagIds.add((short) target.fiducialId);

          // Add observation
          poseObservations.add(
              new PoseObservation(
                  result.getTimestampSeconds(), // Timestamp
                  robotPose, // 3D pose estimate
                  target.poseAmbiguity, // Ambiguity
                  1, // Tag count
                  cameraToTarget.getTranslation().getNorm(), // Average tag distance
                  PoseObservationType.PHOTONVISION)); // Observation type
        }
      }
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
  }
}
