// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonUtils;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogRecord;
import static frc.robot.Constants.Vision.*;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import static frc.robot.Constants.Vision.RED_START_POSE;
import static frc.robot.Constants.Vision.BLUE_START_POSE;   

public class VisionSubsystem extends SubsystemBase {
    // Use a consistent camera name variable
    private static final String CAMERA_NAME = "photonvision";
    private final PhotonCamera photonCamera;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonPoseEstimator photonPoseEstimator;
    
    // Use your pre-defined constant and compute its inverse if needed.
    // // Adjust this depending on whether your estimator expects camera-to-robot or robot-to-camera.
    // private final Transform3d ROBOT_TO_APRILTAG_CAMERA = APRILTAG_CAMERA_TO_ROBOT.inverse();
    
    private Field2d field2d = new Field2d();
    private Field2d estimatedPoseField = new Field2d();

    // Logging fields (set these up as appropriate)
    private DoubleArrayLogEntry visionPose3dLog;
    private DoubleArrayLogEntry robotPose3dLog;
    
    // Standard deviation matrices for estimator uncertainty (ensure these are defined properly)
    private Matrix<N3, N1> curStdDevs;
    private final Matrix<N3, N1> kSingleTagStdDevs = curStdDevs;
    private final Matrix<N3, N1> kMultiTagStdDevs = curStdDevs;

    public VisionSubsystem() {
        // Initialize the PhotonCamera using the name defined in constants or UI
        photonCamera = new PhotonCamera(CAMERA_NAME);
        
        System.out.println("Initializing VisionSubsystem - Is simulation? " + RobotBase.isSimulation());

        // Initialize the AprilTag field layout (for 2025 Reefscape)
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        // Initialize the PhotonPoseEstimator with the field layout, strategy, camera, and transform.
        // Use the transform as required by your pose estimator.
        photonPoseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            APRILTAG_CAMERA_TO_ROBOT // Or use ROBOT_TO_APRILTAG_CAMERA.inverse() if needed.
        );

        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Initialize SmartDashboard components
        SmartDashboard.putData("Vision/Field", field2d);
        SmartDashboard.putData("Vision/EstimatedPose", estimatedPoseField);

        // Initialize logging for AdvantageScope 3D data
        DataLog log = DataLogManager.getLog();
        visionPose3dLog = new DoubleArrayLogEntry(log, "Vision/EstimatedPose3d");
        robotPose3dLog = new DoubleArrayLogEntry(log, "Robot/Pose3d");

        // Start a background thread to update the connection status of the camera
        new Thread(() -> {
            while (true) {
                boolean connected = photonCamera != null && photonCamera.isConnected();
                SmartDashboard.putBoolean("Vision/CameraConnected", connected);
                try { 
                    Thread.sleep(500); 
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }).start();
    }

    public void Init() {
        
            if (RobotBase.isReal()) {
                Alliance alliance = getAlliance();
                Pose2d startPose = (alliance == Alliance.Blue) ? 
                    Constants.Vision.BLUE_START_POSE : Constants.Vision.RED_START_POSE;
                
                resetPose(startPose);
                estimatedPoseField.setRobotPose(startPose);
            }
        }

    public PhotonPoseEstimator getPhotonEstimator() {
        return photonPoseEstimator;
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        
        if (photonCamera == null || !photonCamera.isConnected()) {
            return visionEst;
        }

        var result = photonCamera.getLatestResult();
        if (result.hasTargets()) {
            visionEst = photonPoseEstimator.update(result);
            updateEstimationStdDevs(visionEst, result.getTargets());

            if (visionEst.isPresent()) {
                // Log the estimated 3D pose
                Pose3d estimatedPose3d = visionEst.get().estimatedPose;
                visionPose3dLog.append(pose3dToDoubleArray(estimatedPose3d));
            }
        }
        return visionEst;
    }

    private Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue); // Default to Blue if not set
    }

    public Pose2d getTagToGoal(int tagId) {
        // Replace with your actual logic to compute a Pose2d for the given tagId
        return new Pose2d(0, 0, new Rotation2d(0));
    }

    private double[] pose3dToDoubleArray(Pose3d pose) {
        return new double[] {
            pose.getX(), pose.getY(), pose.getZ(),
            pose.getRotation().getQuaternion().getW(),
            pose.getRotation().getQuaternion().getX(),
            pose.getRotation().getQuaternion().getY(),
            pose.getRotation().getQuaternion().getZ()
        };
    }

    public void logRobotPose3d(Pose3d pose) {
        robotPose3dLog.append(pose3dToDoubleArray(pose));
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            curStdDevs = kSingleTagStdDevs;
            return;
        }

        var estStdDevs = kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        for (var tgt : targets) {
            var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            curStdDevs = kSingleTagStdDevs;
        } else {
            avgDist /= numTags;
            estStdDevs = numTags > 1 ? kMultiTagStdDevs : kSingleTagStdDevs;
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            curStdDevs = estStdDevs;
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public void resetPose(Pose2d newPose) {
        // Update your internal pose state and dashboard if needed.
        field2d.setRobotPose(newPose);
        estimatedPoseField.setRobotPose(newPose);
    }
    

    public void updateField2d(Pose2d robotPose) {
        field2d.setRobotPose(robotPose);
        estimatedPoseField.setRobotPose(getEstimatedGlobalPose()
            .map(est -> est.estimatedPose.toPose2d())
            .orElse(new Pose2d()));

        var visibleTags = photonCamera.getLatestResult().getTargets().stream()
            .map(PhotonTrackedTarget::getFiducialId)
            .toList();
        
        SmartDashboard.putStringArray("Vision/Visible Tags", visibleTags.stream()
            .map(String::valueOf)
            .toArray(String[]::new));

        SmartDashboard.updateValues();
    }

    public void updateOdometry(SwerveSubsystem drivebase) {
        Optional<EstimatedRobotPose> visionEst = getEstimatedGlobalPose();
        visionEst.ifPresent(est -> 
            drivebase.addVisionMeasurement(
                est.estimatedPose.toPose2d(),
                est.timestampSeconds,
                getEstimationStdDevs()
            )
        );
    }

    public PhotonCamera getCamera() {
        return photonCamera;
    }
}
