// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCMDs;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.FloorTarget;
import frc.robot.subsystems.intake.IntakePivotSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToElevatorFloors extends Command {
  /** Creates a new GoToElevatorFloors. */
  ElevatorSubsystem elevatorSubsystem;
  BooleanSupplier pressedUp;
  BooleanSupplier pressedDown;
  int floor;
  Boolean moveFloorUp;
  Boolean moveFloorDown;
  int maxHeight;
  FloorTarget floorTarget;
  public GoToElevatorFloors(ElevatorSubsystem elevatorSubsystem, BooleanSupplier pressedUp, BooleanSupplier pressedDown) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.pressedUp = pressedUp;
    this.pressedDown = pressedDown;
    addRequirements(elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    floor = 0;
    moveFloorUp = true;
    maxHeight = 2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
    if (floor == 0) {
      floorTarget = FloorTarget.GROUND_FLOOR;
      elevatorSubsystem.setTarget(floorTarget);
    }
    if (floor == 1) {
      floorTarget = FloorTarget.MIDDLE_FLOOR;
      elevatorSubsystem.setTarget(floorTarget);
    }
    if (floor == 2) {
      floorTarget = FloorTarget.TOP_FLOOR;
      elevatorSubsystem.setTarget(floorTarget);
    }
    
    SmartDashboard.putNumber("current elevator floor", floor);
    SmartDashboard.putBoolean("pressed up", pressedUp.getAsBoolean());
    SmartDashboard.putBoolean("pressed down", pressedDown.getAsBoolean());
    SmartDashboard.putBoolean("moveFloorDown", moveFloorDown);
    SmartDashboard.putBoolean("moveFloorUp", moveFloorUp);

    
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
