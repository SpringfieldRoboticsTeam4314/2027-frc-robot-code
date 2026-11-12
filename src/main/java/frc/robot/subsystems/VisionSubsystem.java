package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera("front-left-cam");
    private PhotonPoseEstimator photonEstimator;
    private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();

    // Physical position of the camera relative to the center of the robot (in meters & radians)
    private final Transform3d robotToCam = new Transform3d(
        new Translation3d(0.25, 0.0, 0.40),
        new Rotation3d(0, Math.toRadians(-15), 0)
    );

    public VisionSubsystem() {
        try {
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            
            // Fixed constructor: takes only field layout and robot-to-camera transform
            photonEstimator = new PhotonPoseEstimator(
                fieldLayout,
                robotToCam
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTag field layout", e.getStackTrace());
        }
    }

    @Override
    public void periodic() {
        latestEstimatedPose = Optional.empty();

        if (photonEstimator == null) return;

        // Process all unread frames from the camera queue
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            // Use multi-tag estimation first for highest accuracy
            Optional<EstimatedRobotPose> visionPose = photonEstimator.estimateCoprocMultiTagPose(result);
            
            // Fallback to lowest ambiguity single tag if multi-tag isn't available
            if (visionPose.isEmpty()) {
                visionPose = photonEstimator.estimateLowestAmbiguityPose(result);
            }

            if (visionPose.isPresent()) {
                latestEstimatedPose = visionPose;
            }
        }
    }

    /** Returns the newest vision pose estimate calculated during this 20ms cycle, if any. */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return latestEstimatedPose;
    }
}