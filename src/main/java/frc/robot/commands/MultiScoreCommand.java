package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ScoringTarget;
import frc.robot.subsystems.ScoringTargetManager;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.ElevatorCMDs.SetElevatorLevelCommand;
import frc.robot.commands.IntakeMotorCMDs.ESpitCMD;

public class MultiScoreCommand extends SequentialCommandGroup {
  private final Field2d field;

  /**
   * Creates a new MultiScoreCommand.
   *
   * @param vision   the vision subsystem
   * @param drivebase the swerve drive subsystem
   * @param elevator the elevator subsystem
   * @param intake   the intake motor subsystem
   * @param manager  the scoring target manager
   * @param field    the Field2d for path visualization
   */
  public MultiScoreCommand(
      VisionSubsystem vision,
      SwerveSubsystem drivebase,
      ElevatorSubsystem elevator,
      IntakeMotorSubsystem intake,
      ScoringTargetManager manager,
      Field2d field
  ) {
    this.field = field;

    // Gather all the targets in a list
    List<ScoringTarget> allTargets = new ArrayList<>();
    ScoringTarget next;
    while ((next = manager.getNextTarget()) != null) {
      allTargets.add(next);
      manager.callTarget(next);           // reserve it now so getNextTarget won't return it again
      manager.markScored(next);           // mark immediately so getNextTarget moves on
    }
    // Reset manager state for the actual run
    manager.reset();

    // Build one big sequence to score each target
    List<Command> sequence = new ArrayList<>();
    for (ScoringTarget target : allTargets) {
      // 1) Announce target
      sequence.add(Commands.runOnce(() -> {
        manager.callTarget(target);
        SmartDashboard.putString("Picked target", target.toString());
        System.out.println("[MultiScore] Starting target: " + target);
      }));

      // 2) Align to target
      sequence.add(
        new AutoAlignCommand(vision, drivebase, () -> Optional.of(target), field)
      );

      // 3) Move elevator
      sequence.add(
        new SetElevatorLevelCommand(elevator, target.getLevel())
          .beforeStarting(() -> System.out.println(
            "[MultiScore] Elevator to level " + target.getLevel()
          ))
          .withTimeout(2.0)
      );

      // 4) Spit game piece
      sequence.add(
        new ESpitCMD(intake)
          .withTimeout(1.0)
      );

      // 5) Mark scored
      sequence.add(Commands.runOnce(() -> {
        manager.markScored(target);
        System.out.println("[MultiScore] Scored: " + target);
      }));
    }

    // If there were no targets, insert a no-op
    if (sequence.isEmpty()) {
      sequence.add(Commands.none());
    }

    // Finally build the SequentialCommandGroup
    addCommands(sequence.toArray(new Command[0]));
  }
}