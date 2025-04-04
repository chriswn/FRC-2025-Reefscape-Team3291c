package frc.robot;

import static frc.robot.Constants.Vision.*;

import java.util.List;
import java.util.Optional;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionSim {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;
    
    // Simulation components
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    public VisionSim(PhotonCamera cam_in) {
        System.out.println("Initializing VisionSim - Is simulation? " + Robot.isSimulation()); 

        camera = cam_in;
        photonEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            kRobotToCam
        );
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Simulation setup
        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            
          // Load AprilTag field layout using the new method
    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    visionSim.addAprilTags(tagLayout);

            // Configure camera properties
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(320, 240, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(30);  // Reduced from 70 to match your supported mode
            cameraProp.setAvgLatencyMs(30);
            cameraProp.setLatencyStdDevMs(10);
            
            // Create camera simulation
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            
            // Add camera to vision system
            visionSim.addCamera(cameraSim, kRobotToCam);
            
            // Enable streams
            cameraSim.enableDrawWireframe(true);
            cameraSim.enableRawStream(true);
            cameraSim.enableProcessedStream(true);
            
            // Add debug field to dashboard
            SmartDashboard.putData("Vision Sim Field", visionSim.getDebugField());
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        if (camera == null) return visionEst;
        
        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());

            if (Robot.isSimulation() && visionEst.isPresent()) {
                getSimDebugField().getObject("VisionEstimation")
                    .setPose(visionEst.get().estimatedPose.toPose2d());
            }
        }
        return visionEst;
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
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            curStdDevs = kSingleTagStdDevs;
        } else {
            avgDist /= numTags;
            if (numTags > 1) estStdDevs = kMultiTagStdDevs;
            if (numTags == 1 && avgDist > 4) {
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            }
            curStdDevs = estStdDevs;
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    // Simulation methods
    public void simulationPeriodic(Pose2d robotSimPose) {
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.update(robotSimPose);
        }
    }

    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.resetRobotPose(pose);
        }
    }

    public Field2d getSimDebugField() {
        return (Robot.isSimulation() && visionSim != null) ? visionSim.getDebugField() : null;
    }
}