// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto.AutoDirectives.RR;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RR_ToSpeaker extends SequentialCommandGroup {
  /** Creates a new MildAuto. */
  public RR_ToSpeaker(SwerveSubsystem s_sSwerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    TrajectoryConfig config = //needs to be actually configured- either wait for the actual robot to be built or use sysid now, no harm done in the end really
    new TrajectoryConfig(
            Constants.AutoConstants.kMaxSpeedMetersPerSecond,
            Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(Constants.Swerve.swerveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory Trajectory1 =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through an interior waypoint
            List.of(new Translation2d(0.96647, -0.7239),
            new Translation2d(1.93294,-0.9652)),
            // End 1 meter straight ahead of where we started, facing forward
            new Pose2d(1.93294, -1.4478, new Rotation2d(180)),
            config);

    ProfiledPIDController thetaController =
      new ProfiledPIDController(
      Constants.AutoConstants.kPThetaController,
      0,
      0,
      Constants.AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
      new SwerveControllerCommand(
        Trajectory1,
        s_sSwerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        s_sSwerve::setModuleStates,
        s_sSwerve);

    addCommands(
      new InstantCommand(() -> s_sSwerve.resetPoseEstimator(Trajectory1.getInitialPose())),
      swerveControllerCommand);
  }
    
  }
