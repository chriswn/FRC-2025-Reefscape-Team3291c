package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Simulation support for PhotonVision.
 * Provides realistic AprilTag detection simulation for testing vision code without a physical robot.
 */
public class PhotonVisionSimulation {
    
    private final PhotonCamera camera;
    private final VisionSystemSim visionSim;
    private final PhotonCameraSim cameraSim;
    
    /**
     * Creates a new PhotonVisionSimulation.
     * Only functional when running in simulation mode.
     * 
     * @param camera The PhotonCamera to simulate
     */
    public PhotonVisionSimulation(PhotonCamera camera) {
        this.camera = camera;
        
        if (!RobotBase.isSimulation()) {
            System.out.println("PhotonVisionSimulation: Not in simulation mode, skipping initialization");
            visionSim = null;
            cameraSim = null;
            return;
        }
        
        System.out.println("Initializing PhotonVisionSimulation");
        
        // Create vision system simulator
        visionSim = new VisionSystemSim("photonvision_sim");
        
        // Load and add AprilTag field layout
        AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(
            AprilTagFields.k2025ReefscapeAndyMark);
        visionSim.addAprilTags(tagLayout);
        
        // Configure and add camera simulation
        cameraSim = createCameraSim();
        visionSim.addCamera(cameraSim, ROBOT_TO_CAMERA);
        
        // Add debug field to dashboard for visualization
        SmartDashboard.putData("PhotonVision/SimField", visionSim.getDebugField());
        
        System.out.println("PhotonVisionSimulation initialized successfully");
    }
    
    /**
     * Updates the simulation with the current robot pose.
     * Call this periodically (e.g., in simulationPeriodic) to update simulated camera data.
     * 
     * @param robotPose Current robot pose on the field
     */
    public void update(Pose2d robotPose) {
        if (visionSim != null) {
            visionSim.update(robotPose);
        }
    }
    
    /**
     * Resets the simulated robot pose.
     * Useful for testing specific field positions.
     * 
     * @param pose The pose to reset to
     */
    public void resetPose(Pose2d pose) {
        if (visionSim != null) {
            visionSim.resetRobotPose(pose);
        }
    }
    
    /**
     * Gets the debug field for additional visualization.
     * 
     * @return The Field2d used for debug visualization, or null if not in simulation
     */
    public Field2d getDebugField() {
        return (visionSim != null) ? visionSim.getDebugField() : null;
    }
    
    /**
     * Gets the vision system simulator.
     * 
     * @return The VisionSystemSim instance, or null if not in simulation
     */
    public VisionSystemSim getVisionSim() {
        return visionSim;
    }
    
    /**
     * Checks if simulation is active.
     * 
     * @return true if running in simulation mode, false otherwise
     */
    public boolean isSimulationActive() {
        return visionSim != null && cameraSim != null;
    }
    
    // Private helper methods
    
    private PhotonCameraSim createCameraSim() {
        SimCameraProperties cameraProps = new SimCameraProperties();
        
        // Configure camera resolution and FOV
        // 1280x720 resolution with 90 degree FOV
        // IMPORTANT: FOV parameter interpretation may change between PhotonVision versions.
        // Verify this matches your camera specifications and PhotonVision API documentation.
        // Incorrect FOV settings will produce inaccurate simulation results.
        cameraProps.setCalibration(1280, 720, Rotation2d.fromDegrees(90));
        
        // Simulate detection noise (average and std dev error in pixels)
        cameraProps.setCalibError(0.25, 0.08);
        
        // Set framerate (limited by robot loop rate)
        cameraProps.setFPS(30);
        
        // Simulate network/processing latency
        cameraProps.setAvgLatencyMs(30);
        cameraProps.setLatencyStdDevMs(5);
        
        // Create camera simulation
        PhotonCameraSim camSim = new PhotonCameraSim(camera, cameraProps);
        
        // Enable visualization features
        camSim.enableRawStream(true);
        camSim.enableProcessedStream(true);
        camSim.enableDrawWireframe(true);
        
        System.out.println("Camera simulation configured:");
        System.out.println("  Resolution: 1280x720");
        System.out.println("  FOV: 90 degrees");
        System.out.println("  FPS: 30");
        System.out.println("  Latency: 30±5ms");
        
        return camSim;
    }
}
