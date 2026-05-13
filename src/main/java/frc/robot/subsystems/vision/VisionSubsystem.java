// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FieldConstants;
import frc.robot.util.Utils;

/**
 * Vision subsystem for AprilTag-based pose estimation using PhotonVision.
 * Uses getAllUnreadResults() called only once per periodic cycle and
 * caches the results for later use by the subsystem.
 */
public class VisionSubsystem extends SubsystemBase {
  // Functional interface for consuming vision measurements (pose, timestamp, stdDevs)
  // This allows the VisionSubsystem to push measurements to the drive subsystem 
  // without tight coupling
  @FunctionalInterface
  public interface VisionMeasurementConsumer {
    void accept(Pose2d pose, double timestamp, double[] stdDevs);
  }

  // Vision measurement consumer (injected from drive subsystem for pose estimator updates)
  private final VisionMeasurementConsumer visionConsumer;

  // Camera object storage
  private final List<Camera> cameras = new ArrayList<>();

  /**
   * Creates a new VisionSubsystem
   */
  public VisionSubsystem(VisionMeasurementConsumer visionConsumer) {
    // Store the vision consumer for pose estimator updates
    this.visionConsumer = visionConsumer;

    // Exit early if vision is not enabled to avoid unnecessary initialization
    if (!VisionConstants.kEnableVision) {
      Utils.logError("Vision is not enabled! Vision subsystem not initialized");
      return;
    }

    // Get the defined camera configurations
    Map<String, Transform3d> configs = VisionConstants.kCameraConfigs;
    
    // Add each to camera the list
    for (String cameraName : configs.keySet()) {
      try {
        cameras.add(new Camera(cameraName, configs.get(cameraName)));
      } catch (Exception e) {
        Utils.logError("Failed to initialize camera " + cameraName + ": " + e.getMessage());
      }
    }
    
    // Warn if no cameras initialized
    if (cameras.isEmpty()) {
      Utils.logError("Vision enabled but no cameras initialized!");
    }

    // Initialize dashboard values
    SmartDashboard.putData("Vision", this);

    // Output initialization progress
    Utils.logInfo("Vision subsystem initialized");
  }

  @Override
  public void periodic() {
    // CRITICAL: Update each camera's cache BEFORE processing measurements.
    // The camera's updateCache() function should be the ONLY place where 
    // getAllUnreadResults() gets called (once per camera per cycle)!
    for (Camera camera : cameras) {
      try {
        camera.updateCache();
      } catch (Exception e) {
        Utils.logError("Error updating cache for " + camera.getName() + ": " + e.getMessage());
      }
    }
    
    // Process vision measurements using cached data
    updateVisionMeasurements();
  }

  // ==================== Internal State Modifiers ====================

  /**
   * Update vision measurements from cached camera data
   */
  private void updateVisionMeasurements() {
    // If vision is not enabled, skip processing
    if (!isEnabled()) { 
      return;
    }

    // Process each camera's cached unread results
    for (Camera camera : cameras) {
      try {
        // Process ALL cached unread results
        for (PhotonPipelineResult result : camera.getCachedResults()) {
          // Skip further processing if result has no targets
          if (!result.hasTargets())  {
            continue;
          }
          
          // Get pose estimate from multi-tag estimator
          Optional<EstimatedRobotPose> poseResult = camera.getPoseEstimator().estimateCoprocMultiTagPose(result);
          
          // Fallback to lowest ambiguity estimate if multi-tag failed
          if (poseResult.isEmpty()) {            
            poseResult = camera.getPoseEstimator().estimateLowestAmbiguityPose(result);
          };

          // Skip if multi-tag and lowest ambiguity both returned nothing
          if (poseResult.isEmpty()) {
            continue;
          }

          // Get estimated pose
          EstimatedRobotPose estimate = poseResult.get();
          if (estimate == null || estimate.estimatedPose == null) {
            continue;
          }
          
          // Skip further processing if the pose estimate is not valid
          if (!isValidPose(result, estimate))  {
            continue;
          }
          
          // Calculate standard deviations
          double[] stdDevs = calculateStandardDeviations(result);

          // Push vision measurement to drive subsystem via the vision consumer callback
          visionConsumer.accept(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, stdDevs);
        }
      } catch (Exception e) {
        Utils.logError("Error processing vision for " + camera.getName() + ": " + e.getMessage());
      }
    }
  }

  /**
   * Validates a pose estimate to reject obviously incorrect measurements
   * @param result The photon camera pipeline result validate
   * @param estimate The estimated robot pose from the camera
   * @return True if the pose is valid
   */
  private boolean isValidPose(PhotonPipelineResult result, EstimatedRobotPose estimate) {
    // Check for single tag estimates
    if (result.getTargets().size() == 1) {
      // Get the target
      PhotonTrackedTarget target = result.getBestTarget();

      // Ambiguity check
      if (target == null || target.getPoseAmbiguity() > VisionConstants.kPoseAmbiguityThreshold) {
        return false;
      }

      // Area check (tags too small = far away = unreliable)
      if (target.getArea() < VisionConstants.kMinTagAreaPixels) {
        return false;
      }
    }
    
    // Get the 2D pose for boundary checks
    Pose2d pose2d = estimate.estimatedPose.toPose2d();
    
    // Check if pose is within field boundaries
    if (!isPoseOnField(pose2d)) {
      return false;
    }
    
    // Check Z coordinate (robot should be on the ground)
    double z = estimate.estimatedPose.getZ();
    if (Math.abs(z) > VisionConstants.kZMargin) {
      return false;
    }
    
    // If we reach here, the pose is considered valid
    return true;
  }

  /**
   * Calculate standard deviations based on target quality
   * @param result The photon camera pipeline result to calculate standard deviations for
   * @return Array of standard deviations for x, y, and theta
   */
  private double[] calculateStandardDeviations(PhotonPipelineResult result) {
    int numTargets = result.getTargets().size();

    double avgDistance = result.getTargets().stream()
      .mapToDouble(t -> {
        var transform = t.getBestCameraToTarget();
        return transform != null
          ? transform.getTranslation().getNorm()
          : VisionConstants.kMaxDistanceMeters;
      })
      .average()
      .orElse(VisionConstants.kMaxDistanceMeters);

    avgDistance = Math.min(avgDistance, VisionConstants.kMaxDistanceMeters);

    double avgAmbiguity = result.getTargets().stream()
      .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
      .average()
      .orElse(0.0);

    double baseXY = (numTargets == 1)
      ? VisionConstants.kSingleTagBaseXYstdDev
      : VisionConstants.kMultiTagBaseXYstdDev;

    double baseTheta = (numTargets == 1)
      ? VisionConstants.kSingleTagBaseThetaStdDev
      : VisionConstants.kMultiTagBaseThetaStdDev;

    double xyStdDev = baseXY;
    double thetaStdDev = baseTheta;

    // Distance scaling (slightly softer)
    xyStdDev *= (1.0 + (avgDistance * avgDistance / 20.0));
    thetaStdDev *= (1.0 + (avgDistance * avgDistance / 40.0));

    // Tag count scaling (less aggressive)
    double tagFactor = 1.0 / Math.sqrt(numTargets);
    xyStdDev *= tagFactor;
    thetaStdDev *= tagFactor;

    // Ambiguity scaling (earlier + smooth)
    if (avgAmbiguity > 0.05) {
      double scale = 1.0 + (avgAmbiguity * 5.0);
      xyStdDev *= scale;
      thetaStdDev *= scale;
    }

    // Single tag rotation penalty (stronger)
    if (numTargets == 1) {
      thetaStdDev *= 2.5;
    }

    // Clamp to safe bounds
    xyStdDev = MathUtil.clamp(xyStdDev, 0.01, 1.5);
    thetaStdDev = MathUtil.clamp(thetaStdDev, 0.01, Math.PI);

    // Return the calculated standard deviations
    return new double[] {xyStdDev, xyStdDev, thetaStdDev};
  }

  // ==================== Public state accessors ====================

  /**
   * Checks if vision subsystem is enabled (has at least one working camera)
   * @return True if vision is enabled
   */
  public boolean isEnabled() {
    return !cameras.isEmpty() && VisionConstants.kEnableVision;
  }

  /**
   * Check if a given pose is within the field boundaries
   * @param pose The pose to check
   * @return True if the pose is on the field
   */
  public boolean isPoseOnField(Pose2d pose) {
    if (pose == null) {
      return false;
    }
    
    return pose.getX() >= -VisionConstants.kFieldBorderMargin && 
           pose.getX() <= FieldConstants.kFieldLengthMeters + VisionConstants.kFieldBorderMargin &&
           pose.getY() >= -VisionConstants.kFieldBorderMargin && 
           pose.getY() <= FieldConstants.kFieldWidthMeters + VisionConstants.kFieldBorderMargin;
  }

  /**
   * Get a vision system camera by name.
   * @param cameraName Name of the camera
   * @return Optional containing the camera
   */
  public Optional<Camera> getCamera(String cameraName) {
    return cameras.stream()
      .filter(camera -> camera.getName().equals(cameraName))
      .findFirst();
  }

  /**
   * Gets the best target from the specified camera (from cached data)
   * @param cameraName Name of the camera
   * @return Optional containing the best target
   */
  public Optional<PhotonTrackedTarget> getBestTarget(String cameraName) {
    return cameras.stream()
      .filter(camera -> camera.getName().equals(cameraName))
      .findFirst()
      .flatMap(Camera::getBestTarget);
  }

  /**
   * Gets the best target from any camera (from cached data)
   * @return Optional containing the best target
   */
  public Optional<PhotonTrackedTarget> getBestTarget() {
    return cameras.stream()
      .map(Camera::getBestTarget)
      .filter(Optional::isPresent)
      .map(Optional::get)
      .findFirst();
  }

  /**
   * Checks if any camera has visible targets (from cached data)
   * @return True if any targets are visible
   */
  public boolean hasTargets() {
    return cameras.stream().anyMatch(Camera::hasTargets);
  }

  /**
   * Gets the number of visible targets across all cameras (from cached data)
   * @return Total number of visible targets
   */
  public int getTotalTargetCount() {
    return cameras.stream().mapToInt(Camera::getTargetCount).sum();
  }

  /**
   * Gets a list of all camera names
   * @return List of camera names
   */
  public List<String> getCameraNames() {
    return cameras.stream().map(camera -> camera.getName()).toList();
  }

  /**
   * Gets the most recent result for a specific camera (from cached data)
   * @param cameraName Name of the camera
   * @return Most recent PhotonPipelineResult, or empty result if camera not found
   */
  public PhotonPipelineResult getLatestResult(String cameraName) {
    return cameras.stream()
      .filter(camera -> camera.getName().equals(cameraName))
      .findFirst()
      .map(Camera::getLatestResult)
      .orElse(new PhotonPipelineResult());
  }

  /**
   * Initialize Sendable for SmartDashboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Enabled", this::isEnabled, null);
    builder.addBooleanProperty("Has Targets", this::hasTargets, null);
    builder.addDoubleProperty("Total Targets", this::getTotalTargetCount, null);
    
    // Individual camera status - call getLatestResult() in the lambda
    for (Camera camera : cameras) {
      String prefix = "Camera/" + camera.getName() + "/";
    
      builder.addBooleanProperty(prefix + "Connected", () -> camera.getCamera().isConnected(), null);
      builder.addBooleanProperty(prefix + "Has Targets", () -> camera.getLatestResult().hasTargets(), null);
      builder.addDoubleProperty(prefix + "Target Count", () -> camera.getLatestResult().getTargets().size(), null);
      builder.addDoubleProperty(prefix + "Unread Results", () -> camera.getCachedResults().size(), null);
      
      // For best target data, check if targets exist each time
      builder.addDoubleProperty(prefix + "Best Target ID", () -> {
        PhotonPipelineResult r = camera.getLatestResult();
        return r.hasTargets() ? r.getBestTarget().getFiducialId() : -1;
      }, null);
      builder.addDoubleProperty(prefix + "Best Target Distance", () -> {
        PhotonPipelineResult r = camera.getLatestResult();
        return r.hasTargets() ? r.getBestTarget().getBestCameraToTarget().getTranslation().getNorm() : 0;
      }, null);
      builder.addDoubleProperty(prefix + "Best Target Ambiguity", () -> {
        PhotonPipelineResult r = camera.getLatestResult();
        return r.hasTargets() ? r.getBestTarget().getPoseAmbiguity() : 0;
      }, null);
    }
  }
}
