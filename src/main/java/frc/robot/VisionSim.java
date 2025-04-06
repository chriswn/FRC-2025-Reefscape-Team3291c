package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

import static frc.robot.Constants.Vision.*;

public class VisionSim {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private final Field2d field2d = new Field2d();
    private final Field2d estimatedPoseField = new Field2d();
    private Matrix<N3, N1> curStdDevs;
    
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    // AdvantageScope 3D logging
    private final DoubleArrayLogEntry visionPose3dLog;
    private final DoubleArrayLogEntry robotPose3dLog;

    public VisionSim(PhotonCamera cam_in) {
        System.out.println("Initializing VisionSim - Is simulation? " + RobotBase.isSimulation());

        camera = cam_in;
        
        // Initialize with 2025 field layout
        photonEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            kRobotToCam
            //MULTI_TAG_PNP_ON_COPROCESSOR
        );
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Initialize dashboard components
        SmartDashboard.putData("Vision/Field", field2d);
        SmartDashboard.putData("Vision/EstimatedPose", estimatedPoseField);

        // Initialize AdvantageScope 3D logging
        DataLog log = DataLogManager.getLog();
        visionPose3dLog = new DoubleArrayLogEntry(log, "Vision/EstimatedPose3d");
        robotPose3dLog = new DoubleArrayLogEntry(log, "Robot/Pose3d");

        if (RobotBase.isSimulation()) {
            initializeSimulation();
        }

        SmartDashboard.putBoolean("Vision/CameraConnected", false);
    new Thread(() -> {
    while (true) {
        boolean connected = camera != null && camera.isConnected();
        SmartDashboard.putBoolean("Vision/CameraConnected", connected);
        try { Thread.sleep(500); } catch (InterruptedException e) {}
    }
    }).start();
    }

    public void simulationInit() {
        if (RobotBase.isSimulation() && visionSim != null) {
            // Reset simulation pose to a known location with AprilTags
            resetSimPose(new Pose2d(1.5, 1.5, new Rotation2d()));
            visionSim.getDebugField().setRobotPose(new Pose2d(1.5, 1.5, new Rotation2d()));
        }
    }
    private void initializeSimulation() {
        // Create vision system with unique name
        visionSim = new VisionSystemSim("photonvision_sim");
        
        // Load 2025 AprilTag layout
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        visionSim.addAprilTags(tagLayout);

        // Configure camera properties
        configureCameraSim();
    }

    private void configureCameraSim() {
        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.25, 0.08);
        cameraProp.setFPS(30);
        cameraProp.setAvgLatencyMs(30);
        cameraProp.setLatencyStdDevMs(5);
        
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);
        visionSim.addCamera(cameraSim, kRobotToCam);
        // Force initial update with robot near tags
        // visionSim.update(new Pose2d(1.5, 1.5, new Rotation2d()));

        // Add debug field to dashboard
        SmartDashboard.putData("PhotonVision/SimField", visionSim.getDebugField());
        
        System.out.println("PhotonVision simulation initialized for camera: " + camera.getName());
    }

    public PhotonPoseEstimator getPhotonEstimator() {
        return photonEstimator;
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

            if (visionEst.isPresent()) {
                // Log 3D pose to AdvantageScope
                Pose3d estimatedPose3d = visionEst.get().estimatedPose;
                visionPose3dLog.append(pose3dToDoubleArray(estimatedPose3d));

                if (RobotBase.isSimulation()) {
                    visionSim.getDebugField().getObject("Estimation")
                        .setPose(estimatedPose3d.toPose2d());
                }
            }
        }
        return visionEst;
    }
    public Pose2d getTagToGoal(int tagId) {

        // Replace with the actual logic to compute the Pose2d for the given tagId

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
            estStdDevs = numTags > 1 ? kMultiTagStdDevs : kSingleTagStdDevs;
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
            curStdDevs = estStdDevs;
        }
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    public void updateField2d(Pose2d robotPose) {
        field2d.setRobotPose(robotPose);
        estimatedPoseField.setRobotPose(getEstimatedGlobalPose()
            .map(est -> est.estimatedPose.toPose2d())
            .orElse(new Pose2d()));
        
        // Force update dashboard
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
    
    public void simulationPeriodic(Pose2d robotSimPose) {
        if (RobotBase.isSimulation() && visionSim != null) {
            visionSim.update(robotSimPose);
            logRobotPose3d(new Pose3d(
                robotSimPose.getX(),
                robotSimPose.getY(),
                0,
                new Rotation3d(0, 0, robotSimPose.getRotation().getRadians())
            ));
            
            // Update diagnostic telemetry
            SmartDashboard.putBoolean("Vision/CameraConnected", camera.isConnected());
            SmartDashboard.putNumber("Vision/TargetsFound", 
                camera.getLatestResult().getTargets().size());
        }
    }

    public void resetSimPose(Pose2d pose) {
        if (RobotBase.isSimulation() && visionSim != null) {
            visionSim.resetRobotPose(pose);
        }
    }

    public Field2d getSimDebugField() {
        return (RobotBase.isSimulation() && visionSim != null) ? 
            visionSim.getDebugField() : null;
    }

    public PhotonCamera getCamera() {
        return camera;
    }
}