package frc.robot.commands;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ColorChanger;


public class ClimbCMD extends Command {
  /** Creates a new MoveIntakeMotorCMD. */
  ColorChanger colorChanger;
  ClimberSubsystem climberSubsystem;
  DoubleSupplier positiveSupplier;
  DoubleSupplier negativeSupplier;
  BooleanSupplier aToggle;
  BooleanSupplier colorToggled;
  private int hasChanged;
  private int colorToggleNumber;
  public ClimbCMD(
    ClimberSubsystem climberSubsystem,
    ColorChanger colorChanger,
    DoubleSupplier positiveSupplier,
    DoubleSupplier negativeSupplier,
    BooleanSupplier aToggle,
    BooleanSupplier colorToggled
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climberSubsystem = climberSubsystem;
    this.colorChanger = colorChanger;
    addRequirements(climberSubsystem, colorChanger);

    this.positiveSupplier = positiveSupplier;
    this.negativeSupplier = negativeSupplier;
    this.aToggle = aToggle;
    this.colorToggled = colorToggled;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.hasChanged = 0;
    this.colorToggleNumber = 2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isAToggled = aToggle.getAsBoolean();
    boolean colorToggle = colorToggled.getAsBoolean();

    if (isAToggled && this.hasChanged == 0) {
        this.hasChanged = 1;
    }

    if (this.hasChanged == 1 && !isAToggled) {
        this.hasChanged = 2;
      }

    if (this.hasChanged == 2 && isAToggled) {
        this.hasChanged = 3;
      }

    if (this.hasChanged == 3 && !isAToggled) {
        this.hasChanged = 0; 
      }

      if (colorToggle && this.colorToggleNumber == 0) {
        this.colorToggleNumber = 1;
    }

    if (this.colorToggleNumber == 1 && !colorToggle) {
        this.colorToggleNumber = 2;
      }

    if (this.colorToggleNumber == 2 && colorToggle) {
        this.colorToggleNumber = 3;
      }

    if (this.colorToggleNumber == 3 && !colorToggle) {
        this.colorToggleNumber = 0; 
      }

    if (this.hasChanged == 0) {
      if (this.colorToggleNumber == 2) {
    colorChanger.setOrange();
      }
    double positiveSpeed = positiveSupplier.getAsDouble();
    double negativeSpeed = negativeSupplier.getAsDouble();
    climberSubsystem.setClimberTogether(0, negativeSpeed);
    }
    else if (this.hasChanged == 2) {
      if (colorToggleNumber == 2) {
    colorChanger.setPurple();
      }
    double positiveSpeed = positiveSupplier.getAsDouble();
    double negativeSpeed = negativeSupplier.getAsDouble();
    climberSubsystem.setClimberIndividual(positiveSpeed, -negativeSpeed);
    }
    
    
}

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

