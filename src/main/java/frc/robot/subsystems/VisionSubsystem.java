// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT;

// import java.io.IOException;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import org.photonvision.targeting.PhotonTrackedTarget;
// import org.photonvision.targeting.PhotonPipelineResult;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import org.photonvision.PhotonUtils;
// import java.util.Optional;
// import org.photonvision.EstimatedRobotPose;

// public class VisionSubsystem extends SubsystemBase {
//     private final PhotonCamera photonCamera;
//     private static final String CameraName = "photonvision";
//     private AprilTagFieldLayout aprilTagFieldLayout;
//     private PhotonPoseEstimator photonPoseEstimator;
//     private final Transform3d ROBOT_TO_APRILTAG_CAMERA;

//     public VisionSubsystem() {
//         // Initialize the camera using the class field
//         photonCamera = new PhotonCamera(CameraName);
        
//         // Calculate the robot-to-camera transform
//         ROBOT_TO_APRILTAG_CAMERA = APRILTAG_CAMERA_TO_ROBOT.inverse();

//         try {
//             // Load the field layout
//             aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
//             aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            
//             // Initialize the pose estimator
//             photonPoseEstimator = new PhotonPoseEstimator(
//                 aprilTagFieldLayout, 
//                 PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//                 photonCamera, 
//                 ROBOT_TO_APRILTAG_CAMERA
//             );
            
//             photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
//         } catch (IOException e) {
//             DriverStation.reportError("Failed to load AprilTagFieldLayout: " + e.getMessage(), false);
//             photonPoseEstimator = null;
//         } catch (Exception e) {
//             DriverStation.reportError("Unexpected error initializing PhotonPoseEstimator: " + e.getMessage(), false);
//             photonPoseEstimator = null;
//         }
//     }

//     @Override 
//     public void periodic() {
//         // Query the latest result from PhotonVision
//         var result = photonCamera.getLatestResult();

//         // Check if the latest result has any targets.
//         if (result.hasTargets()) {
//             // Get the current best target.
//             PhotonTrackedTarget target = result.getBestTarget();

//             // Get information from target.
//             double yaw = target.getYaw();
//             double pitch = target.getPitch();
//             double area = target.getArea();
//             double skew = target.getSkew();
//             int targetID = target.getFiducialId();

//             SmartDashboard.putNumber("Vision/Yaw", yaw);
//             SmartDashboard.putNumber("Vision/Pitch", pitch);
//             SmartDashboard.putNumber("Vision/Area", area);
//             SmartDashboard.putNumber("Vision/Skew", skew);
//             SmartDashboard.putNumber("Vision/Target ID", targetID);

//             // Check for AprilTag ID and calculate field-relative robot pose
//             if (targetID != -1 && aprilTagFieldLayout != null) {
//                 Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(targetID);
//                 if (tagPose.isPresent()) {
//                     Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
//                         target.getBestCameraToTarget(),
//                         tagPose.get(),
//                         ROBOT_TO_APRILTAG_CAMERA.inverse() // Convert back to camera-to-robot
//                     );
                    
//                     double distanceToTarget = PhotonUtils.getDistanceToPose(
//                         robotPose.toPose2d(), 
//                         tagPose.get().toPose2d()
//                     );
                    
//                     SmartDashboard.putNumber("Vision/Distance to Target", distanceToTarget);
//                 }
//             }
//         }
//     }

//     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
//         if (photonPoseEstimator != null) {
//             photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
//             return photonPoseEstimator.update();
//         }
//         return Optional.empty();
//     }
// }