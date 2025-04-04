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
    private final Field2d field2d = new Field2d();
    private final Field2d estimatedPoseField = new Field2d();
    private Matrix<N3, N1> curStdDevs;
    
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    public VisionSim(PhotonCamera cam_in) {
        System.out.println("Initializing VisionSim - Is simulation? " + Robot.isSimulation());

        camera = cam_in;
        
        // Initialize with 2025 field layout
        photonEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            kRobotToCam
        );
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        SmartDashboard.putData("Vision/Field", field2d);
        SmartDashboard.putData("Vision/EstimatedPose", estimatedPoseField);


        if (Robot.isSimulation()) {
            initializeSimulation();
        }
    }

    private void initializeSimulation() {
        // Create vision system with unique name
        visionSim = new VisionSystemSim("photonvision_sim");
        
        // Load 2025 AprilTag layout
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        visionSim.addAprilTags(tagLayout);

        // Configure camera properties
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(320, 240, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.25, 0.08);
        cameraProp.setFPS(30);
        cameraProp.setAvgLatencyMs(30);
        cameraProp.setLatencyStdDevMs(5);

        // Create camera simulation
        cameraSim = new PhotonCameraSim(camera, cameraProp);

        // Add camera to vision system
        visionSim.addCamera(cameraSim, kRobotToCam);

        // Configure streams - ORDER MATTERS HERE
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        // Force initial update with robot near tags
        visionSim.update(new Pose2d(1.5, 1.5, new Rotation2d()));

        // Add debug field to dashboard
        SmartDashboard.putData("PhotonVision/SimField", visionSim.getDebugField());
        
        System.out.println("PhotonVision simulation initialized for camera: " + camera.getName());
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        
        if (camera == null || !camera.isConnected()) {
            return visionEst;
        }

        var result = camera.getLatestResult();
        if (result.hasTargets()) {
            visionEst = photonEstimator.update(result);
            updateEstimationStdDevs(visionEst, result.getTargets());

            if (Robot.isSimulation() && visionEst.isPresent()) {
                visionSim.getDebugField().getObject("Estimation")
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