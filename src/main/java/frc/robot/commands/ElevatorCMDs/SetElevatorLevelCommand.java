
package frc.robot.commands.ElevatorCMDs;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.FloorTarget;

/**
 * Moves the elevator to a given scoring level (1-3) and finishes when reached.
 */
public class SetElevatorLevelCommand extends Command {
    private final ElevatorSubsystem elevator;
    private final int level;
    private FloorTarget target;

    public SetElevatorLevelCommand(ElevatorSubsystem elevator, int level) {
        this.elevator = elevator;
        this.level = level;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // Map level (1-3) to ElevatorSubsystem.FloorTarget
        switch (level) {
            case 1:
                target = FloorTarget.SECOND_FLOOR;
                break;
            case 2:
                target = FloorTarget.THIRD_FLOOR;
                break;
            case 3:
                target = FloorTarget.FOURTH_FLOOR;
                break;
            default:
                target = FloorTarget.GROUND_FLOOR;
                break;
        }
        // Command the elevator to move to the desired floor
        elevator.setTarget(target);
    }

    @Override
    public boolean isFinished() {
        // Check if elevator has reached the target height
        double height = elevator.floorTargetToHeight(target);
        return elevator.ifAtFloor(height);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop elevator motion
        elevator.stopElevator();
    }
}
