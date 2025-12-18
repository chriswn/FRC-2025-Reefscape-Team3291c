/**
 * PhotonVision-based vision system for FRC robot localization and AprilTag tracking.
 * 
 * <h2>Overview</h2>
 * This package provides a complete vision solution using PhotonVision for:
 * <ul>
 *   <li>AprilTag detection and tracking</li>
 *   <li>Robot pose estimation on the field</li>
 *   <li>Vision-odometry fusion for accurate localization</li>
 *   <li>Simulation support for testing without hardware</li>
 * </ul>
 * 
 * <h2>Main Components</h2>
 * <ul>
 *   <li>{@link frc.robot.subsystems.vision.PhotonVisionSubsystem} - Main subsystem for vision operations</li>
 *   <li>{@link frc.robot.subsystems.vision.PhotonVisionSimulation} - Simulation support</li>
 *   <li>{@link frc.robot.subsystems.vision.VisionConstants} - Configuration constants</li>
 *   <li>{@link frc.robot.subsystems.vision.VisionUtils} - Utility methods</li>
 * </ul>
 * 
 * <h2>Quick Start</h2>
 * <pre>{@code
 * // In RobotContainer
 * private final PhotonVisionSubsystem vision = new PhotonVisionSubsystem();
 * 
 * // Initialize with starting pose
 * public void robotInit() {
 *     vision.initialize();
 * }
 * 
 * // Update odometry periodically
 * public void robotPeriodic() {
 *     vision.updateOdometry(drivebase);
 * }
 * }</pre>
 * 
 * <h2>Configuration</h2>
 * Edit {@link frc.robot.subsystems.vision.VisionConstants} to configure:
 * <ul>
 *   <li>Camera name and physical mounting position</li>
 *   <li>Pose estimation standard deviations</li>
 *   <li>Auto-alignment PID gains and constraints</li>
 *   <li>Target ambiguity thresholds</li>
 * </ul>
 * 
 * @see <a href="https://docs.photonvision.org">PhotonVision Documentation</a>
 * @see <a href="https://docs.wpilib.org/en/stable/docs/software/vision-processing">WPILib Vision Processing</a>
 */
package frc.robot.subsystems.vision;
