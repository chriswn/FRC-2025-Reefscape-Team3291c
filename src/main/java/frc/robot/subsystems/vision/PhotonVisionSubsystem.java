package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * PhotonVision-based vision subsystem for AprilTag detection and robot pose estimation.
 * 
 * This subsystem integrates with PhotonVision to:
 * - Detect AprilTags on the field
 * - Estimate robot pose using camera measurements
 * - Update robot odometry with vision measurements
 * - Provide diagnostics and telemetry
 */
public class PhotonVisionSubsystem extends SubsystemBase {
    
    private final PhotonCamera photonCamera;
    private final PhotonPoseEstimator photonPoseEstimator;
    
    private final Field2d field2d = new Field2d();
    private final Field2d estimatedPoseField = new Field2d();
    
    // Logging for AdvantageScope 3D visualization
    private final DoubleArrayLogEntry visionPose3dLog;
    private final DoubleArrayLogEntry robotPose3dLog;
    
    // Current standard deviations for pose estimation
    private Matrix<N3, N1> currentStdDevs = SINGLE_TAG_STD_DEVS;
    
    /**
     * Creates a new PhotonVisionSubsystem.
     * Initializes the camera, pose estimator, and telemetry.
     */
    public PhotonVisionSubsystem() {
        this(CAMERA_NAME);
    }
    
    /**
     * Creates a new PhotonVisionSubsystem with a custom camera name.
     * 
     * @param cameraName Name of the PhotonVision camera as configured in the UI
     */
    public PhotonVisionSubsystem(String cameraName) {
        photonCamera = new PhotonCamera(cameraName);
        
        System.out.println("Initializing PhotonVisionSubsystem");
        System.out.println("  Camera: " + cameraName);
        System.out.println("  Is Real Robot: " + RobotBase.isReal());
        
        // Initialize PhotonPoseEstimator with field layout and strategy
        photonPoseEstimator = new PhotonPoseEstimator(
            APRILTAG_FIELD_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            ROBOT_TO_CAMERA
        );
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        // Initialize SmartDashboard components
        SmartDashboard.putData("Vision/Field", field2d);
        SmartDashboard.putData("Vision/EstimatedPose", estimatedPoseField);
        
        // Initialize logging for AdvantageScope 3D data
        DataLog log = DataLogManager.getLog();
        visionPose3dLog = new DoubleArrayLogEntry(log, "Vision/EstimatedPose3d");
        robotPose3dLog = new DoubleArrayLogEntry(log, "Robot/Pose3d");
        
        // Start background thread to monitor camera connection
        startCameraStatusThread();
        
        System.out.println("PhotonVisionSubsystem initialized successfully");
    }
    
    /**
     * Initialize the subsystem with alliance-specific starting pose.
     * Should be called during robot initialization.
     */
    public void initialize() {
        if (RobotBase.isReal()) {
            Alliance alliance = getAlliance();
            Pose2d startPose = (alliance == Alliance.Blue) ? 
                BLUE_START_POSE : RED_START_POSE;
            
            resetPose(startPose);
            estimatedPoseField.setRobotPose(startPose);
            
            System.out.println("Vision initialized for " + alliance + " alliance at " + startPose);
        }
    }
    
    /**
     * Gets the PhotonPoseEstimator for advanced usage.
     * 
     * @return The PhotonPoseEstimator instance
     */
    public PhotonPoseEstimator getPhotonEstimator() {
        return photonPoseEstimator;
    }
    
    /**
     * Gets the estimated global pose of the robot based on AprilTag detections.
     * 
     * @return Optional containing the estimated pose if targets are detected, empty otherwise
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        if (photonCamera == null || !photonCamera.isConnected()) {
            return Optional.empty();
        }
        
        var result = photonCamera.getLatestResult();
        if (!result.hasTargets()) {
            SmartDashboard.putString("Vision/TargetStatus", "No Targets Detected");
            return Optional.empty();
        }
        
        Optional<EstimatedRobotPose> visionEst = photonPoseEstimator.update(result);
        SmartDashboard.putString("Vision/TargetStatus", "Targets Detected");
        updateEstimationStdDevs(visionEst, result.getTargets());
        
        if (visionEst.isPresent()) {
            // Log the estimated 3D pose for AdvantageScope
            Pose3d estimatedPose3d = visionEst.get().estimatedPose;
            visionPose3dLog.append(pose3dToDoubleArray(estimatedPose3d));
        }
        
        return visionEst;
    }
    
    /**
     * Gets the current estimation standard deviations.
     * These values affect how much the pose estimator trusts vision measurements.
     * 
     * @return Matrix containing standard deviations for x, y, and rotation
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return currentStdDevs;
    }
    
    /**
     * Gets the PhotonCamera instance for direct access.
     * 
     * @return The PhotonCamera instance
     */
    public PhotonCamera getCamera() {
        return photonCamera;
    }
    
    /**
     * Resets the displayed robot pose on the field.
     * 
     * @param newPose The new pose to display
     */
    public void resetPose(Pose2d newPose) {
        field2d.setRobotPose(newPose);
        estimatedPoseField.setRobotPose(newPose);
    }
    
    /**
     * Updates the Field2d displays with current robot and vision poses.
     * 
     * @param robotPose The current robot pose from odometry
     */
    public void updateField2d(Pose2d robotPose) {
        field2d.setRobotPose(robotPose);
        estimatedPoseField.setRobotPose(getEstimatedGlobalPose()
            .map(est -> est.estimatedPose.toPose2d())
            .orElse(new Pose2d()));
        
        // Display visible AprilTag IDs
        var visibleTags = photonCamera.getLatestResult().getTargets().stream()
            .map(PhotonTrackedTarget::getFiducialId)
            .toList();
        
        SmartDashboard.putString("Vision/VisibleTags", visibleTags.toString());
        SmartDashboard.updateValues();
    }
    
    /**
     * Updates robot odometry with vision measurements.
     * This should be called periodically to fuse vision data with wheel odometry.
     * 
     * @param drivebase The swerve drive subsystem to update
     */
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
    
    /**
     * Logs a robot pose in 3D for AdvantageScope visualization.
     * 
     * @param pose The 3D pose to log
     */
    public void logRobotPose3d(Pose3d pose) {
        robotPose3dLog.append(pose3dToDoubleArray(pose));
    }
    
    /**
     * Checks if the camera is currently connected.
     * 
     * @return true if camera is connected, false otherwise
     */
    public boolean isCameraConnected() {
        return photonCamera != null && photonCamera.isConnected();
    }
    
    // Private helper methods
    
    private Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }
    
    private double[] pose3dToDoubleArray(Pose3d pose) {
        return VisionUtils.pose3dToDoubleArray(pose);
    }
    
    /**
     * Updates standard deviations based on number of tags and distance.
     * This heuristic adjusts trust in vision measurements dynamically.
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, 
            List<PhotonTrackedTarget> targets) {
        
        if (estimatedPose.isEmpty()) {
            currentStdDevs = SINGLE_TAG_STD_DEVS;
            return;
        }
        
        var estStdDevs = SINGLE_TAG_STD_DEVS;
        int numTags = 0;
        double avgDist = 0;
        
        // Calculate number of valid tags and average distance
        for (var target : targets) {
            var tagPose = photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) continue;
            
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }
        
        if (numTags == 0) {
            currentStdDevs = SINGLE_TAG_STD_DEVS;
        } else {
            avgDist /= numTags;
            
            // Use better std devs for multi-tag detections
            estStdDevs = numTags > 1 ? MULTI_TAG_STD_DEVS : SINGLE_TAG_STD_DEVS;
            
            // Increase uncertainty with distance
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            currentStdDevs = estStdDevs;
        }
    }
    
    private void startCameraStatusThread() {
        new Thread(() -> {
            while (true) {
                boolean connected = isCameraConnected();
                SmartDashboard.putBoolean("Vision/CameraConnected", connected);
                SmartDashboard.putString("Vision/CameraStatus", 
                    connected ? "Connected" : "Not Connected");
                
                try {
                    Thread.sleep(CAMERA_STATUS_UPDATE_RATE_MS);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }).start();
    }
}
