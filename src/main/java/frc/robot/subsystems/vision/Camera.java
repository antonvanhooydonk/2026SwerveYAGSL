package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;

import frc.robot.Constants.FieldConstants;

public class Camera {
  private final String name;
  private final Transform3d robotToCamera;
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  
  // Cached data from current periodic cycle
  private List<PhotonPipelineResult> cachedResults = new ArrayList<>();
  
  /**
   * Construct a new Camera instance
   * @param config The camera configuration
   */
  public Camera(String cameraName, Transform3d robotToCamera) {
    // Assign to fields
    this.name = cameraName;
    this.robotToCamera = robotToCamera;
    this.camera = new PhotonCamera(cameraName);
    this.poseEstimator = new PhotonPoseEstimator(FieldConstants.kFieldLayout, robotToCamera);
  }
      
  /**
   * Updates cached results - CALL ONLY ONCE PER PERIODIC CYCLE
   */
  public void updateCache() {
    cachedResults = camera.getAllUnreadResults();
  }
      
  /**
   * Gets the most recent result from cache
   */
  public PhotonPipelineResult getLatestResult() {
    if (!cachedResults.isEmpty()) {
      return cachedResults.get(cachedResults.size() - 1);
    }
    return new PhotonPipelineResult();
  }
  
  /**
   * Checks if any cached results have targets
   */
  public boolean hasTargets() {
    if (!cachedResults.isEmpty()) {
      return cachedResults.stream().anyMatch(PhotonPipelineResult::hasTargets);
    }
    return false;
  }
      
  /**
   * Gets target count from most recent cached result
   */
  public int getTargetCount() {
    PhotonPipelineResult latestResult = getLatestResult();
    return latestResult.getTargets().size();
  }
      
  /**
   * Gets best target from most recent cached result
   */
  public Optional<PhotonTrackedTarget> getBestTarget() {
    PhotonPipelineResult latestResult = getLatestResult();
    return latestResult.hasTargets() ? Optional.of(latestResult.getBestTarget()) : Optional.empty();
  }

  /**
   * Gets the camera name
   * @return The camera's name
   */
  public String getName() {
    return name;
  }

  /**
   * Gets the camera's Transform3d
   * @return The camera's Transform3d
   */
  public Transform3d getRobotToCamera() {
    return robotToCamera;
  }

  /**
   * Gets the PhotonCamera instance
   * @return
   */
  public PhotonCamera getCamera() {
    return camera;
  }

  /**
   * Gets the PhotonPoseEstimator instance
   * @return
   */
  public PhotonPoseEstimator getPoseEstimator() {
    return poseEstimator;
  }

  /**
   * Gets the cached results
   * @return
   */
  public List<PhotonPipelineResult> getCachedResults() {
    return new ArrayList<>(cachedResults);
  }
  
  /**
   * Checks if camera is connected
   * @return True if camera is connected
   */
  public boolean isConnected() {
    return camera.isConnected();
  }

  /**
   * Gets the total number of results in cache
   */
  public int getCachedResultCount() {
    return cachedResults.size();
  }

  /**
  * Gets the timestamp of the most recent result
  */
  public double getLatestTimestamp() {
    PhotonPipelineResult latest = getLatestResult();
    return latest.getTimestampSeconds();
  }

  /**
   * Sets the active pipeline index (if multiple pipelines are configured)
   */
  public void setPipelineIndex(int index) {
    camera.setPipelineIndex(index);
  }

  /**
  * Gets the current pipeline index (if multiple pipelines are configured)
  */
  public int getPipelineIndex() {
    return camera.getPipelineIndex();
  }

  /**
   * Sets camera LED mode
   */
  public void setLED(boolean enabled) {
    camera.setLED(enabled ? VisionLEDMode.kOn : VisionLEDMode.kOff);
  }
}
