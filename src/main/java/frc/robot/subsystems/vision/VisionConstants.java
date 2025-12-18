package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * Constants for PhotonVision-based vision system.
 * Contains camera configuration, field layout, and pose estimation parameters.
 */
public final class VisionConstants {
    
    // Camera Configuration
    /**
     * Name of the PhotonVision camera as configured in the PhotonVision UI.
     * IMPORTANT: Change this to match your team's camera configuration.
     */
    public static final String CAMERA_NAME = "limelight-front-3291";
    
    /**
     * Physical location of the camera on the robot, relative to the center of the robot.
     * Measures in meters: X (forward/back), Y (left/right), Z (up/down)
     * and rotation: Roll, Pitch, Yaw
     */
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(0.5, 0.0, 0.5),  // X forward, Y left/right, Z up
        new Rotation3d(0.0, 0.0, 0.0)       // Roll, Pitch, Yaw
    );
    
    // Field Layout
    /**
     * AprilTag field layout for the 2025 Reefscape game.
     */
    public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = 
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    
    // Pose Estimation Standard Deviations
    /**
     * Standard deviations for pose estimation when using a single AprilTag.
     * Format: [x (meters), y (meters), rotation (radians)]
     * Higher values = less trust in single tag measurements
     */
    public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = 
        VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(30));
    
    /**
     * Standard deviations for pose estimation when using multiple AprilTags.
     * Format: [x (meters), y (meters), rotation (radians)]
     * Lower values = more trust in multi-tag measurements
     */
    public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = 
        VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(10));
    
    // Starting Positions
    /**
     * Starting pose for Blue Alliance (field coordinates in meters).
     */
    public static final Pose2d BLUE_START_POSE = 
        new Pose2d(8.05, 4.5, Rotation2d.fromDegrees(180));
    
    /**
     * Starting pose for Red Alliance (field coordinates in meters).
     */
    public static final Pose2d RED_START_POSE = 
        new Pose2d(9.5, 2.5, Rotation2d.fromDegrees(0));
    
    // Target Configuration
    /**
     * Default AprilTag ID to track for auto-alignment commands.
     */
    public static final int DEFAULT_TARGET_TAG_ID = 12;
    
    /**
     * Minimum target ambiguity threshold.
     * Targets with higher ambiguity will be discarded.
     * Range: 0.0 (perfect) to 1.0 (maximum ambiguity)
     */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    
    // Auto-Alignment PID Constants
    /**
     * Proportional gain for auto-alignment translation (X/Y).
     */
    public static final double AUTO_ALIGN_TRANSLATION_KP = 1.5;
    
    /**
     * Derivative gain for auto-alignment translation (X/Y).
     */
    public static final double AUTO_ALIGN_TRANSLATION_KD = 0.2;
    
    /**
     * Proportional gain for auto-alignment rotation.
     */
    public static final double AUTO_ALIGN_ROTATION_KP = 2.0;
    
    /**
     * Derivative gain for auto-alignment rotation.
     */
    public static final double AUTO_ALIGN_ROTATION_KD = 0.1;
    
    // Motion Constraints
    /**
     * Maximum translation velocity for auto-alignment (meters/second).
     */
    public static final double AUTO_ALIGN_MAX_VELOCITY = 3.0;
    
    /**
     * Maximum translation acceleration for auto-alignment (meters/second²).
     */
    public static final double AUTO_ALIGN_MAX_ACCELERATION = 2.0;
    
    /**
     * Maximum rotational velocity for auto-alignment (radians/second).
     */
    public static final double AUTO_ALIGN_MAX_ANGULAR_VELOCITY = 8.0;
    
    /**
     * Maximum rotational acceleration for auto-alignment (radians/second²).
     */
    public static final double AUTO_ALIGN_MAX_ANGULAR_ACCELERATION = 8.0;
    
    // Tolerances
    /**
     * Position tolerance for auto-alignment (meters).
     */
    public static final double AUTO_ALIGN_POSITION_TOLERANCE = 0.05;
    
    /**
     * Rotation tolerance for auto-alignment (radians).
     */
    public static final double AUTO_ALIGN_ROTATION_TOLERANCE = Units.degreesToRadians(1.5);
    
    // Camera Status Update Rate
    /**
     * How often to check camera connection status (milliseconds).
     */
    public static final int CAMERA_STATUS_UPDATE_RATE_MS = 500;
    
    private VisionConstants() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
