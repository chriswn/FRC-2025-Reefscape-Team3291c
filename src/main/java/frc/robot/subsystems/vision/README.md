# PhotonVision Module

A complete, production-ready PhotonVision integration for FRC robots featuring AprilTag detection, robot pose estimation, and vision-odometry fusion.

## Features

- ✅ **AprilTag Detection** - Automatic detection and tracking of AprilTags on the field
- ✅ **Pose Estimation** - Multi-tag and single-tag pose estimation with automatic fallback
- ✅ **Vision-Odometry Fusion** - Seamless integration with swerve drive odometry
- ✅ **Simulation Support** - Full PhotonVision simulation for testing without hardware
- ✅ **Dynamic Standard Deviations** - Adjusts trust in vision measurements based on tag count and distance
- ✅ **AdvantageScope Integration** - 3D pose logging for visualization
- ✅ **Comprehensive Telemetry** - Real-time diagnostics via SmartDashboard

## Quick Start

### 1. Basic Usage

```java
// In RobotContainer.java
private final PhotonVisionSubsystem vision = new PhotonVisionSubsystem();

public void robotInit() {
    vision.initialize();  // Sets starting pose based on alliance
}

public void robotPeriodic() {
    vision.updateOdometry(drivebase);  // Fuses vision with wheel odometry
}

// Optional: Clean up resources when robot is shutting down
@Override
public void close() {
    vision.close();  // PhotonVisionSubsystem implements AutoCloseable
}
```

### 2. With Simulation

```java
// In RobotContainer.java
private final PhotonCamera camera = new PhotonCamera("your-camera-name");
private final PhotonVisionSubsystem vision = new PhotonVisionSubsystem("your-camera-name");
private final PhotonVisionSimulation visionSim = new PhotonVisionSimulation(camera);

public void simulationPeriodic() {
    if (visionSim.isSimulationActive()) {
        visionSim.update(drivebase.getPose());
    }
}
```

## Configuration

All configuration is centralized in `VisionConstants.java`:

### Camera Configuration
```java
public static final String CAMERA_NAME = "your-camera-name";
public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
    new Translation3d(0.5, 0.0, 0.5),  // X, Y, Z in meters
    new Rotation3d(0.0, 0.0, 0.0)      // Roll, Pitch, Yaw in radians
);
```

### Pose Estimation Tuning
```java
// Trust in single-tag measurements [x, y, rotation]
public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = 
    VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(30));

// Trust in multi-tag measurements (more accurate)
public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = 
    VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(10));
```

### Auto-Alignment PID
```java
public static final double AUTO_ALIGN_TRANSLATION_KP = 1.5;
public static final double AUTO_ALIGN_TRANSLATION_KD = 0.2;
public static final double AUTO_ALIGN_ROTATION_KP = 2.0;
public static final double AUTO_ALIGN_ROTATION_KD = 0.1;
```

## Architecture

### Core Components

| Component | Purpose |
|-----------|---------|
| `PhotonVisionSubsystem` | Main subsystem managing camera, pose estimation, and odometry fusion |
| `PhotonVisionSimulation` | Simulation support for testing without hardware |
| `VisionConstants` | Centralized configuration for all vision parameters |
| `VisionUtils` | Helper methods for common vision operations |

### Key Features Explained

#### Dynamic Standard Deviations
The system automatically adjusts how much it trusts vision measurements based on:
- **Number of tags**: More tags = lower standard deviations (more trust)
- **Distance to tags**: Closer tags = lower standard deviations (more trust)

```java
// Automatically calculated in PhotonVisionSubsystem
if (numTags > 1) {
    stdDevs = MULTI_TAG_STD_DEVS;
} else {
    stdDevs = SINGLE_TAG_STD_DEVS;
}
stdDevs = stdDevs.times(1 + (avgDistance² / 30));
```

#### Pose Estimation Strategies
1. **Primary**: `MULTI_TAG_PNP_ON_COPROCESSOR` - Best accuracy with multiple tags
2. **Fallback**: `LOWEST_AMBIGUITY` - Most reliable single tag when multi-tag fails

## Example Commands

### Auto-Align to AprilTag
```java
// In RobotContainer.java
driverController.a().whileTrue(
    new AutoAlignCommand(vision, drivebase, targetTagId)
);
```

### Chase AprilTag
```java
// In RobotContainer.java
driverController.b().whileTrue(
    new ChaseTag2(vision, drivebase)
);
```

## Telemetry

The vision system publishes extensive telemetry to SmartDashboard:

| Key | Description |
|-----|-------------|
| `Vision/CameraConnected` | Boolean indicating camera connectivity |
| `Vision/CameraStatus` | String status message |
| `Vision/TargetStatus` | Current target detection status |
| `Vision/VisibleTags` | List of currently visible AprilTag IDs |
| `Vision/Field` | Field2d showing robot pose |
| `Vision/EstimatedPose` | Field2d showing vision-estimated pose |
| `Vision/EstimatedPose3d` | 3D pose for AdvantageScope (log entry) |
| `Robot/Pose3d` | Robot's 3D pose (log entry) |

## Simulation

The simulation system provides realistic AprilTag detection for testing:

```java
// Configure camera properties (matches real camera)
- Resolution: 1280x720
- FOV: 90 degrees diagonal
- FPS: 30
- Latency: 30±5ms
- Detection noise: 0.25px avg, 0.08px std dev
```

View the simulation:
- `PhotonVision/SimField` on SmartDashboard shows the simulated field view

## Troubleshooting

### Camera Not Connecting
1. Verify camera name matches PhotonVision configuration
2. Check network connection to camera
3. Monitor `Vision/CameraConnected` on SmartDashboard

### Poor Pose Estimates
1. Verify `ROBOT_TO_CAMERA` transform is accurate
2. Tune standard deviations in `VisionConstants`
3. Check tag distance (closer is better)
4. Ensure good lighting conditions

### Simulation Not Working
1. Verify you're calling `PhotonVisionSimulation.update()` in `simulationPeriodic()`
2. Check that robot pose is being updated
3. Ensure AprilTag field layout matches game year

## Migration Guide

### From Legacy VisionSubsystem
The old `VisionSubsystem` now wraps `PhotonVisionSubsystem`. To migrate:

```java
// Old (still works but deprecated)
import frc.robot.subsystems.VisionSubsystem;
private final VisionSubsystem vision = new VisionSubsystem();

// New (recommended)
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
private final PhotonVisionSubsystem vision = new PhotonVisionSubsystem();
```

### From Constants.Vision
```java
// Old
import static frc.robot.Constants.Vision.*;

// New
import static frc.robot.subsystems.vision.VisionConstants.*;
```

## Advanced Usage

### Custom Camera Name
```java
PhotonVisionSubsystem vision = new PhotonVisionSubsystem("my-camera-name");
```

### Manual Pose Updates
```java
Optional<EstimatedRobotPose> estimate = vision.getEstimatedGlobalPose();
if (estimate.isPresent()) {
    // Use pose for custom logic
    Pose2d visionPose = estimate.get().estimatedPose.toPose2d();
    double timestamp = estimate.get().timestampSeconds;
}
```

### Target Validation
```java
import frc.robot.subsystems.vision.VisionUtils;

// Check if target meets ambiguity threshold
if (VisionUtils.isTargetValid(target)) {
    // Use target
}

// Custom ambiguity threshold
if (VisionUtils.isTargetValid(target, 0.15)) {
    // Use target with stricter threshold
}
```

## Resources

- [PhotonVision Documentation](https://docs.photonvision.org)
- [WPILib Vision Processing](https://docs.wpilib.org/en/stable/docs/software/vision-processing)
- [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) - For 3D visualization

## Credits

Based on PhotonVision example code and best practices from the FRC community.
Refactored and enhanced for the 2025 Reefscape season.
