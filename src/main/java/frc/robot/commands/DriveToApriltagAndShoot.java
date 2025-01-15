// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.DriveToApriltag;
import frc.robot.subsystems.IntakeMotorSubsystem;
import frc.robot.subsystems.LauncherSub;
import frc.robot.subsystems.PreferencesSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToApriltagAndShoot extends SequentialCommandGroup {
  /** Creates a new DriveToApriltagAndShoot. */
  public DriveToApriltagAndShoot(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, IntakeMotorSubsystem intakeMotorSubsystem, LauncherSub launcherSub, int desiredId, PreferencesSubsystem preferencesSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToApriltag(swerveSubsystem, visionSubsystem),
      new LaunchWheelCMD(launcherSub, preferencesSubsystem).withTimeout(5)
    );
  }
}
