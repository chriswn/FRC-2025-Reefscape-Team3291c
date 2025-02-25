// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.IntakeMotorCMDs;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;



public class IntakeCMD extends Command {
  IntakeMotorSubsystem intakeMotorSubsystem;
  Boolean hadCoral;
  /** Creates a new IntakeMotorCMD. */
  public IntakeCMD(IntakeMotorSubsystem intakeMotorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeMotorSubsystem = intakeMotorSubsystem;
   
    addRequirements(intakeMotorSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeMotorSubsystem.isActive = false;

    if (intakeMotorSubsystem.hasCoral()) {
      hadCoral = true;
    }
    else {
      hadCoral = false;
    }
  }
 


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (hadCoral) {
        intakeMotorSubsystem.moveIntakeMotor(intakeMotorSubsystem.ejectSpeed);
      }
      else if (!hadCoral) {
        intakeMotorSubsystem.moveIntakeMotor(intakeMotorSubsystem.intakeSpeed);
      }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeMotorSubsystem.isActive = false;
    intakeMotorSubsystem.stopIntakeMotorSubsystem();//stops it
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!hadCoral && intakeMotorSubsystem.hasCoral()){
      Timer.delay(Constants.Intake.intakeTimeToStop);
      return true;
    }
    else if (hadCoral && !intakeMotorSubsystem.hasCoral()) {
      Timer.delay(Constants.Intake.ejectTimeToStop);
      return true;
    }
    else {
      return false;
    }
  }
}











