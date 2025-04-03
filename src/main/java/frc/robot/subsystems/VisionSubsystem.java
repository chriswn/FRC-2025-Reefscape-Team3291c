// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;
import org.photonvision.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

public class VisionSubsystem extends SubsystemBase{
 private PhotonCamera camera;
 private static final String CAMERA_NAME = "photonvision";
 private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

 private Transform3d cameraToRobot; 

 //constructor
  public VisionSubsystem(){
      PhotonCamera camera = new PhotonCamera("photonvision");
      // The field from AprilTagFields will be different depending on the game.
   try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            e.printStackTrace();  // Handle error loading the field layout
        }
      // Initialize cameraToRobot (you need to define this transform based on your robot's configuration)
      cameraToRobot = new Transform3d();  // Replace this with actual robot configuration
    }

 @Override 
 public void periodic(){
  
  // Query the latest result from PhotonVision
  var result7= camera.getLatestResult();

  // Check if the latest result has any targets.
  boolean hasTargets = result.hasTargets();

  if(result.hasTargets()){
 // Get a list of all targets detected by the camera
  List<PhotonTrackedTarget> targets = result.getTargets();
  
  // Get the current best target.
  PhotonTrackedTarget target = result.getBestTarget();

  // Get information from target.
double yaw = target.getYaw();
double pitch = target.getPitch();
double area = target.getArea();
double skew = target.getSkew();
Transform2d pose = target.getCameraToTarget();
List<TargetCorner> corners = target.getCorners();

// Get information from target.
int targetID = target.getFiducialId();
double poseAmbiguity = target.getPoseAmbiguity();
Transform3d bestCameraToTarget = target.getBestCameraToTarget();
Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

  // if (bestTarget.getFiducialId() != -1) {
  //               int targetID = bestTarget.getFiducialId();
  //               double poseAmbiguity = bestTarget.getPoseAmbiguity();
  //               Transform3d bestCameraToTarget = target.getBestCameraToTarget();
  //               Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
  //           }

 SmartDashboard.putNumber("Yaw", yaw);
 SmartDashboard.putNumber("Pitch", pitch);
 SmartDashboard.putNumber("Area", area);
 SmartDashboard.putNumber("Skew", skew);

   // Check for AprilTag ID and calculate field-relative robot pose
  int targetID = target.getFiducialId();
            if (aprilTagFieldLayout.getTagPose(targetID).isPresent()) {
                Pose3d tagPose = aprilTagFieldLayout.getTagPose(targetID).get();
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                        target.getBestCameraToTarget(),
                        tagPose,
                        cameraToRobot
                );
            
    // You can also calculate the robot's field-relative pose using the target's data
      if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
  Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
  double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, targetPose);
  SmartDashboard.putNumber("Distance to Target", distanceToTarget);
      }
            }
  }
 
 }


}