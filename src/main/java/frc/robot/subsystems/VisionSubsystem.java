package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera1 = new PhotonCamera("camera 1");
    private final PhotonCamera camera2 = new PhotonCamera("camera 2");

    private PhotonPoseEstimator photonEstimator1;
    private PhotonPoseEstimator photonEstimator2;

    private final List<EstimatedRobotPose> latestEstimatedPoses = new ArrayList<>();

    // Physical position of camera 1 relative to the center of the robot (meters & radians)
    private final Transform3d robotToCam1 = new Transform3d(
        new Translation3d(0.25, 0.20, 0.40),
        new Rotation3d(0, Math.toRadians(-15), Math.toRadians(15))
    );

    // Physical position of camera 2 relative to the center of the robot (meters & radians)
    private final Transform3d robotToCam2 = new Transform3d(
        new Translation3d(0.25, -0.20, 0.40),
        new Rotation3d(0, Math.toRadians(-15), Math.toRadians(-15))
    );

    public VisionSubsystem() {
        try {
            AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

            photonEstimator1 = new PhotonPoseEstimator(fieldLayout, robotToCam1);
            photonEstimator2 = new PhotonPoseEstimator(fieldLayout, robotToCam2);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load AprilTag field layout", e.getStackTrace());
        }
    }

    @Override
    public void periodic() {
        latestEstimatedPoses.clear();

        if (photonEstimator1 == null || photonEstimator2 == null) return;

        processCamera(camera1, photonEstimator1);
        processCamera(camera2, photonEstimator2);
    }

    private void processCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionPose = estimator.estimateCoprocMultiTagPose(result);

            if (visionPose.isEmpty()) {
                visionPose = estimator.estimateLowestAmbiguityPose(result);
            }

            visionPose.ifPresent(latestEstimatedPoses::add);
        }
    }

    /** Returns all vision pose estimates calculated during this 20ms cycle across both cameras. */
    public List<EstimatedRobotPose> getEstimatedGlobalPoses() {
        return latestEstimatedPoses;
    }
}