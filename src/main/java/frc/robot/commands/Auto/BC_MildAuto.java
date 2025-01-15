// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;
import frc.robot.commands.Auto.AutoDirectives.BC.BC_Out;
import frc.robot.commands.LaunchNoteCMD;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LauncherSub;
import frc.robot.subsystems.PreferencesSubsystem;
import frc.robot.subsystems.IntakeMotorSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BC_MildAuto extends SequentialCommandGroup {
  /** Creates a new BL_MildAuto. */
  public BC_MildAuto(SwerveSubsystem swerveSubsystem, LauncherSub launcherSub, IntakeMotorSubsystem intakeMotorSubsystem, PreferencesSubsystem preferencesSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LaunchNoteCMD(intakeMotorSubsystem, launcherSub, preferencesSubsystem).withTimeout(5),
      new BC_Out(swerveSubsystem)
    );
  }
}
