// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;

/**
 * Compatibility wrapper for the new vision subsystem.
 * 
 * @deprecated Use {@link PhotonVisionSubsystem} directly instead.
 */
@Deprecated
public class VisionSubsystem extends SubsystemBase {
    
    private final PhotonVisionSubsystem visionSubsystem;
    public VisionSubsystem() {
        visionSubsystem = new PhotonVisionSubsystem();
        System.out.println("VisionSubsystem: Using compatibility wrapper. " +
            "Consider migrating to PhotonVisionSubsystem directly.");
    }

    public void Init() {
        visionSubsystem.initialize();
    }

    public PhotonPoseEstimator getPhotonEstimator() {
        return visionSubsystem.getPhotonEstimator();
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return visionSubsystem.getEstimatedGlobalPose();
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return visionSubsystem.getEstimationStdDevs();
    }

    public void resetPose(Pose2d newPose) {
        visionSubsystem.resetPose(newPose);
    }

    public void updateField2d(Pose2d robotPose) {
        visionSubsystem.updateField2d(robotPose);
    }

    public void updateOdometry(SwerveSubsystem drivebase) {
        visionSubsystem.updateOdometry(drivebase);
    }

    public PhotonCamera getCamera() {
        return visionSubsystem.getCamera();
    }
    
    public void logRobotPose3d(Pose3d pose) {
        visionSubsystem.logRobotPose3d(pose);
    }
}
