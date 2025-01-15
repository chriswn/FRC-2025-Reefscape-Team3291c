// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LauncherSub;

public class ParadeCommand extends Command {
  /** Creates a new ParadeCommand. */
  LauncherSub launcherSub;
  ClimberSubsystem climberSubsystem;
  Double startTime;
  Timer timer;
  public ParadeCommand(ClimberSubsystem climberSubsystem, LauncherSub launcherSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcherSub = launcherSub;
    this.climberSubsystem = climberSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Timer.getFPGATimestamp() - startTime < 1.5){
      climberSubsystem.setClimberTogether(0.15,0.05);
    }
    if (Timer.getFPGATimestamp() - startTime > 1.5 && Timer.getFPGATimestamp() - startTime < 3){
      climberSubsystem.setClimberTogether(0.05,0.15);
    }
    if (Timer.getFPGATimestamp() - startTime > 3 && Timer.getFPGATimestamp() - startTime < 4.5){
      climberSubsystem.setClimberTogether(0.15,0.05);
    }
    timer.reset();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
