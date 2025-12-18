package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Utility methods for vision operations.
 */
public final class VisionUtils {
    
    private VisionUtils() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
    
    /**
     * Calculates the robot's target pose relative to an AprilTag.
     * 
     * @param tagId The AprilTag ID
     * @param offsetTransform Transform from the tag to the desired goal position
     * @return Optional containing the target pose if the tag exists in the field layout
     */
    public static Optional<Pose2d> getTagGoalPose(int tagId, Transform3d offsetTransform) {
        Optional<Pose3d> tagPose = VisionConstants.APRILTAG_FIELD_LAYOUT.getTagPose(tagId);
        
        if (tagPose.isEmpty()) {
            return Optional.empty();
        }
        
        Pose3d goalPose3d = tagPose.get().transformBy(offsetTransform);
        return Optional.of(goalPose3d.toPose2d());
    }
    
    /**
     * Checks if a target is valid based on ambiguity threshold.
     * 
     * @param target The target to check
     * @return true if the target's ambiguity is below the threshold
     */
    public static boolean isTargetValid(PhotonTrackedTarget target) {
        double ambiguity = target.getPoseAmbiguity();
        return ambiguity != -1 && ambiguity <= VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
    }
    
    /**
     * Checks if a target is valid based on a custom ambiguity threshold.
     * 
     * @param target The target to check
     * @param maxAmbiguity Maximum acceptable ambiguity
     * @return true if the target's ambiguity is below the threshold
     */
    public static boolean isTargetValid(PhotonTrackedTarget target, double maxAmbiguity) {
        double ambiguity = target.getPoseAmbiguity();
        return ambiguity != -1 && ambiguity <= maxAmbiguity;
    }
    
    /**
     * Converts a Pose3d to a double array for logging.
     * Format: [x, y, z, qw, qx, qy, qz] (quaternion representation)
     * 
     * @param pose The 3D pose to convert
     * @return Array representation suitable for AdvantageScope 3D visualization
     */
    public static double[] pose3dToDoubleArray(Pose3d pose) {
        return new double[] {
            pose.getX(), 
            pose.getY(), 
            pose.getZ(),
            pose.getRotation().getQuaternion().getW(),
            pose.getRotation().getQuaternion().getX(),
            pose.getRotation().getQuaternion().getY(),
            pose.getRotation().getQuaternion().getZ()
        };
    }
    
    /**
     * Calculates the distance from a pose to an AprilTag.
     * 
     * @param robotPose Current robot pose
     * @param tagId AprilTag ID
     * @return OptionalDouble containing the distance in meters if tag exists, empty otherwise
     */
    public static java.util.OptionalDouble getDistanceToTag(Pose2d robotPose, int tagId) {
        Optional<Pose3d> tagPose = VisionConstants.APRILTAG_FIELD_LAYOUT.getTagPose(tagId);
        
        if (tagPose.isEmpty()) {
            return java.util.OptionalDouble.empty();
        }
        
        double distance = robotPose.getTranslation()
            .getDistance(tagPose.get().toPose2d().getTranslation());
        
        return java.util.OptionalDouble.of(distance);
    }
}
