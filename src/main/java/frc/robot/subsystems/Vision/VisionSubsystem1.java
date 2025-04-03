package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTracedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;

public class VisionSubsystem1 extends SubsystemBase {
    // Setting up the camera
    PhotonCamera camera = new PhotonCamera("photonvision");
    
    // The field from AprilTagFields will be different depending on the game.
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);  
    
    @Override
    public void periodic() {
        // Getting the result from the PhotonVision pipeline
        var result = camera.getLatestResult();

        // Checking if the result has any targets
        boolean hasTargets = result.hasTargets();

        if (hasTargets) {
            // Get a list of currently tracked targets.
            List<PhotonTracedTarget> targets = result.getTargets();
            
            // Get the current best target.
            PhotonTrackedTarget target = result.getBestTarget();

            // Get information from target
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getArea();
            double skew = target.getSkew();
            Transform2d pose = target.getCameraToTarget();
            List<TargetCorner> corners = target.getCorners();

            int targetID = target.getFiducialId();
            double poseAmbiguity = target.getPoseAmbiguity();
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

            // Calculate robot's field-relative pose
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(targetID);
            if (tagPose.isPresent()) {
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                        bestCameraToTarget,
                        tagPose.get(),
                        new Transform3d());  // You may want to adjust camera-to-robot transform here

                // Calculate distance to the target
                double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, new Pose3d());

                // Calculate translation to target
                Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
                        distanceToTarget, Rotation2d.fromDegrees(-target.getYaw()));

                Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose, new Pose3d());

                // Update SmartDashboard with useful info
                SmartDashboard.putNumber("Target Yaw", yaw);
                SmartDashboard.putNumber("Target Pitch", pitch);
                SmartDashboard.putNumber("Target Area", area);
                SmartDashboard.putNumber("Target Distance", distanceToTarget);
            }
        }

        // Set driver mode to on
        camera.setDriverMode(true);

        // Change pipeline to 2
        camera.setPipelineIndex(2);

        // Get the pipeline latency
        double latencySeconds = result.getLatencyMillis() / 1000.0;
        SmartDashboard.putNumber("Latency", latencySeconds);
    }

    // Method to get the estimated robot pose based on previous estimated pose
    public Optional<Pose2d> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
                aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, new Transform3d());
        
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        Optional<EstimatedRobotPose> estimatedRobotPose = photonPoseEstimator.update();

        if (estimatedRobotPose.isPresent()) {
            return Optional.of(estimatedRobotPose.get().estimatedPose.toPose2d());
        }
        return Optional.empty();
    }
}
