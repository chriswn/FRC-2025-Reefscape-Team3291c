// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCMDs;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.FloorTarget;
import frc.robot.subsystems.intake.IntakePivotSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToFloor extends Command {
  /** Creates a new GoToElevatorFloors. */
  ElevatorSubsystem elevatorSubsystem;
  IntakePivotSubsystem intakePivotSubsystem;
  BooleanSupplier pressedUp;
  BooleanSupplier pressedDown;
  BooleanSupplier startButton;
  Boolean startButtonReady = true;
  Boolean startButtonPressed = false;
  BooleanSupplier aButton;
  Boolean aButtonReady = true;
  Boolean aButtonPressed = false;
  int floor = 0;
  Boolean moveFloorUp;
  Boolean moveFloorDown;
  int maxHeight = 3;
  FloorTarget floorTarget;
  public GoToFloor(ElevatorSubsystem elevatorSubsystem, IntakePivotSubsystem intakePivotSubsystem, BooleanSupplier pressedUp, BooleanSupplier pressedDown, BooleanSupplier startButton, BooleanSupplier aButton) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.intakePivotSubsystem = intakePivotSubsystem;
    this.pressedUp = pressedUp;
    this.pressedDown = pressedDown;
    this.startButton = startButton;
    this.aButton = aButton;
    addRequirements(elevatorSubsystem);
    addRequirements(intakePivotSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public GoToFloor(ElevatorSubsystem elevatorSubsystem, IntakePivotSubsystem intakePivotSubsystem, BooleanSupplier pressedUp, BooleanSupplier pressedDown, BooleanSupplier startButton, BooleanSupplier aButton, int floor) {
    this(elevatorSubsystem, intakePivotSubsystem, pressedUp, pressedDown, startButton, aButton);
    this.floor = floor;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moveFloorUp = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (startButton.getAsBoolean()) {
      if (startButtonReady) {
        startButtonPressed = !startButtonPressed;
        startButtonReady = false;
      }
    }
    else {
      startButtonReady = true;
    }

    if (aButton.getAsBoolean()) {
      if (aButtonReady) {
        aButtonPressed = !aButtonPressed;
        aButtonReady = false;
      }
    }
    else {
      aButtonReady = true;
    }

    if (pressedUp.getAsBoolean() && moveFloorUp && floor < maxHeight) {
      moveFloorUp = false;
      floor++;
    }
    else if (!pressedUp.getAsBoolean()) {
      moveFloorUp = true;
    }

    if (pressedDown.getAsBoolean() && moveFloorDown && floor > 0) {
      moveFloorDown = false;
      floor--;
    }
    else if (!pressedDown.getAsBoolean()) {
      moveFloorDown = true;
    }

    elevatorSubsystem.algaeMode = aButtonPressed;

    if (floor == 0) {
      if (!startButtonPressed) {
      elevatorSubsystem.setTarget(FloorTarget.GROUND_FLOOR);
      }
      if (elevatorSubsystem.elevatorEncoder.get()/Constants.Elevator.encoderTicksPerRotation < Constants.Elevator.heightOfHeadBang) {
        intakePivotSubsystem.pivot_target = IntakePivotSubsystem.PivotTarget.STOW;
      }
      
    }
    else if (floor == 1) {
      if (aButtonPressed) {
        intakePivotSubsystem.pivot_target = IntakePivotSubsystem.PivotTarget.GROUND;
      } else {
        intakePivotSubsystem.pivot_target = IntakePivotSubsystem.PivotTarget.MIDLEVELS;
      }
      if (!startButtonPressed) {
        elevatorSubsystem.setTarget(FloorTarget.SECOND_FLOOR);
      }    
    }
    else if (floor == 2) {
      if (aButtonPressed) {
        intakePivotSubsystem.pivot_target = IntakePivotSubsystem.PivotTarget.GROUND;
      } 
      else if (intakePivotSubsystem.midLevelsAngle < Constants.Elevator.angleToAvoidHeadBang && elevatorSubsystem.elevatorEncoder.get()/Constants.Elevator.encoderTicksPerRotation < Constants.Elevator.heightOfHeadBang) {
        intakePivotSubsystem.pivot_target = IntakePivotSubsystem.PivotTarget.TOPLEVEL;
      }
      else {
        intakePivotSubsystem.pivot_target = IntakePivotSubsystem.PivotTarget.MIDLEVELS;
      }
      if (!startButtonPressed) {      
        elevatorSubsystem.setTarget(FloorTarget.THIRD_FLOOR);
        } 
     }
    else if (floor == 3) {
      intakePivotSubsystem.pivot_target = IntakePivotSubsystem.PivotTarget.TOPLEVEL;
      if (!startButtonPressed) {
        elevatorSubsystem.setTarget(FloorTarget.FOURTH_FLOOR);
      } 
    }
    
    SmartDashboard.putNumber("current elevator floor", floor);
    SmartDashboard.putBoolean("pressed up", pressedUp.getAsBoolean());
    SmartDashboard.putBoolean("pressed down", pressedDown.getAsBoolean());
    SmartDashboard.putBoolean("moveFloorDown", moveFloorDown);
    SmartDashboard.putBoolean("moveFloorUp", moveFloorUp);
    SmartDashboard.putBoolean("startButton", startButton.getAsBoolean());
    SmartDashboard.putBoolean("startButtonPressed", startButtonPressed);
    SmartDashboard.putBoolean("startButtonReady", startButtonReady);
  
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
