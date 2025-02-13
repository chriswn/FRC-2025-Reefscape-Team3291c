// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.IntakeMotorCMDs;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;


public class EjectCMD extends Command {
  IntakeMotorSubsystem intakeMotorSubsystem;
 // double startTime = Timer.getFPGATimestamp();
  /** Creates a new Eject. */
  public EjectCMD(IntakeMotorSubsystem intakeMotorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeMotorSubsystem = intakeMotorSubsystem;
    addRequirements(intakeMotorSubsystem);
    //this.startTime = startTime;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //intakeMotorSubsystem.moveIntakeMotor(-Constants.intake.ejectSpeed);
   
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeMotorSubsystem.moveIntakeMotor(Constants.Intake.ejectSpeed);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeMotorSubsystem.stopIntakeMotorSubsystem();//stops it


  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
 }



