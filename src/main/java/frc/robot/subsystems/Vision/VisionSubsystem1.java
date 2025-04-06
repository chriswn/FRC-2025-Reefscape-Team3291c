// package frc.robot.subsystems.Vision;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonUtils;
// import org.photonvision.estimation.TargetModel;
// import org.photonvision.simulation.VisionSystemSim;
// import org.photonvision.simulation.VisionTargetSim;
// import org.photonvision.targeting.PhotonTrackedTarget;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.TargetCorner;
// import org.photonvision.targeting.PhotonPipelineMetadata;


// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import java.util.List;
// import java.util.Optional;

// public class VisionSubsystem1 extends SubsystemBase {
//     // Setting up the camera
//     private final PhotonCamera camera = new PhotonCamera("photonvision");
//     VisionSystemSim visionSim = new VisionSystemSim("main");

//     // The field from AprilTagFields will be different depending on the game.
//     private final AprilTagFieldLayout aprilTagFieldLayout;
//     private  PhotonPoseEstimator photonPoseEstimator; 

//     // A 0.5 x 0.25 meter rectangular target
//     TargetModel targetModel = new TargetModel(0.5, 0.25);   
//     // The camera's transform relative to the robot
    
//     Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
//     Transform3d cameraToRobot = new Transform3d(); // Replace this with actual robot configuration
//     Transform3d robotToCamera = cameraToRobot.inverse();

//     public VisionSubsystem1() {
//         try {
//             aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
//         } catch (Exception e) {
//             throw new RuntimeException("Failed to load AprilTagFieldLayout", e);
//         }
//     }
    
//     @Override
//     public void periodic() {
//         // Getting the result from the PhotonVision pipeline
//         PhotonPipelineResult result = camera.getLatestResult();

//         // Checking if the result has any targets
//         boolean hasTargets = result.hasTargets();

//         if (hasTargets) {
//             // Get a list of currently tracked targets.
//             List<PhotonTrackedTarget> targets = result.getTargets();
            
//             // Get the current best target.
//             PhotonTrackedTarget target = result.getBestTarget();

//             // Get information from target
//             double yaw = target.getYaw();
//             double pitch = target.getPitch();
//             double area = target.getArea();
//             double skew = target.getSkew();
//             Transform3d pose = target.getBestCameraToTarget();
//             List<TargetCorner> corners = target.getDetectedCorners();

//             int targetID = target.getFiducialId();
//             double poseAmbiguity = target.getPoseAmbiguity();
//             Transform3d bestCameraToTarget = target.getBestCameraToTarget();
//             Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

//             // Calculate robot's field-relative pose
//             Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(targetID);
//             if (tagPose.isPresent()) {
//                 // You should replace this with your actual camera-to-robot transform
//                 Transform3d cameraToRobot = new Transform3d();
                
//                 Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
//                         bestCameraToTarget,
//                         tagPose.get(),
//                         cameraToRobot);

//                 // Calculate distance to the target
//                 double distanceToTarget = PhotonUtils.getDistanceToPose(
//                         robotPose.toPose2d(), tagPose.get().toPose2d());

//                 // Calculate translation to target
//                 Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
//                         distanceToTarget, Rotation2d.fromDegrees(-target.getYaw()));

//                 Rotation2d targetYaw = PhotonUtils.getYawToPose(
//                         robotPose.toPose2d(), tagPose.get().toPose2d());

//                 // Update SmartDashboard with useful info
//                 SmartDashboard.putNumber("Vision/Target Yaw", yaw);
//                 SmartDashboard.putNumber("Vision/Target Pitch", pitch);
//                 SmartDashboard.putNumber("Vision/Target Area", area);
//                 SmartDashboard.putNumber("Vision/Target Distance", distanceToTarget);
//                 SmartDashboard.putNumber("Vision/Target ID", targetID);
//             }
//         }
//         }

//         // Set driver mode to on (if you want to switch to driver mode)
//         // camera.setDriverMode(true);

//         // Change pipeline if needed (example)
//         // camera.setPipelineIndex(2);

//         // Get the pipeline latency
        

//     // Method to get the estimated robot pose based on previous estimated pose
//     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
//         photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
//         return photonPoseEstimator.update(null);
//     }
// }
