package com.arc852.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class Limelight extends SubsystemBase {
  private final PhotonCamera cam = new PhotonCamera("limelight");
  private final AprilTagFieldLayout aprilTagFieldLayout;

  {
    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  private final PhotonPoseEstimator poseEstimator =
      new PhotonPoseEstimator(
          aprilTagFieldLayout,
          PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
          cam,
          new Transform3d());

  public Limelight() {}
}
