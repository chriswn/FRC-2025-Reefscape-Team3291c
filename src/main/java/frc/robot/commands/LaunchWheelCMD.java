// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LauncherSub;
import frc.robot.subsystems.PreferencesSubsystem;


public class LaunchWheelCMD extends Command {
  /** Creates a new LaunchWheelCMD. */
  LauncherSub launcherSub;
  PreferencesSubsystem preferencesSubsystem;
  double speed;
  public LaunchWheelCMD(LauncherSub launcherSub, PreferencesSubsystem preferencesSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launcherSub = launcherSub;
    this.preferencesSubsystem = preferencesSubsystem;
    addRequirements(launcherSub);
   
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //launcherSub.setSpeed(Constants.launchSpeed);
    //preferencesSubsystem.PreferencesSubsystem("launchSpeed", Constants.launchSpeed);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this.speed += Constants.intake.launchNoteTimeInSecs * Constants.launcherTargetVoltage / 50;
    // if (this.speed >= Constants.launcherTargetVoltage) {
    //   this.speed = Constants.launcherTargetVoltage;
    // }
    // launcherSub.setLaunchWheelsVoltage(-this.speed, this.speed);
    launcherSub.setSpeed(preferencesSubsystem.launchSpeed);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcherSub.stopLauncherSub();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}



