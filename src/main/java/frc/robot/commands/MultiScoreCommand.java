package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.ElevatorCMDs.SetElevatorLevelCommand;
import frc.robot.commands.IntakeMotorCMDs.ESpitCMD;

public class MultiScoreCommand extends SequentialCommandGroup {
  public MultiScoreCommand(
      VisionSubsystem vision,
      SwerveSubsystem drivebase,
      ElevatorSubsystem elevator,
      IntakeMotorSubsystem intake,
      ScoringTargetManager manager
  ) {
    // Gather all the targets in a list
    List<ScoringTarget> allTargets = new ArrayList<>();
    ScoringTarget next;
    while ((next = manager.getNextTarget()) != null) {
      allTargets.add(next);
      manager.callTarget(next);           // reserve it now so getNextTarget won't return it again
      manager.markScored(next);           // mark immediately so getNextTarget moves on
    }
    // Reset manager state for the actual run (optional)
    manager.reset();

    // Build one big sequence to score each target
    List<Command> sequence = new ArrayList<>();
    for (ScoringTarget target : allTargets) {
      // Announce
      sequence.add(Commands.runOnce(() -> {
        manager.callTarget(target);
        SmartDashboard.putString("Picked target", target.toString());
        System.out.println("[MultiScore] Starting target: " + target);
      }));
      // 1) Align
      sequence.add(new AutoAlignCommand(vision, drivebase, () -> Optional.of(target)));
      // 2) Elevate
      sequence.add(new SetElevatorLevelCommand(elevator, target.getLevel())
      .beforeStarting(() -> 
      System.out.println("[MultiScore] Elevator to level " + target.getLevel()))
      .withTimeout(2.0)
      );
      // 3) Spit
      sequence.add(new ESpitCMD(intake).withTimeout(1.0));
      // 4) Mark scored
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
